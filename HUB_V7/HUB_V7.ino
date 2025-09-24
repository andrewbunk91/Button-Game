// ================== HUB_V7.ino ==================
// Transport: ESP-NOW (station mode)
// Board: ESP32-WROOM-32
//
// Features:
// - Receives BTN_REGISTER (button_id, ffa, mac[]) from each button on boot
// - Closes a short registration window automatically, then starts a round
// - If ALL registered buttons have ffa=true -> FFA mode
//   * LED ON for all at start; first valid press per MAC counts; then lockout
// - Else -> Sequence mode
//   * Lights only the expected button's LED; advances one-by-one
// - Sends Hub->Button commands: HUB_LED_SET, HUB_FEEDBACK, HUB_ALL_FLASH
// - Tracks per-MAC last_press_id to ignore duplicates
// - Simple end-of-round reset and re-open registration for the next round
//
// LED pins on the hub board are kept for local debug flashes if desired.
// ===================================================

#include <esp_now.h>
#include <WiFi.h>

// ---------- Optional local debug LEDs ----------
#define GREEN_LED 12
#define RED_LED   14

static inline void flashGreen(uint16_t ms = 120) {
  digitalWrite(GREEN_LED, HIGH); delay(ms);
  digitalWrite(GREEN_LED, LOW);
}
static inline void flashRed(uint16_t ms = 120) {
  digitalWrite(RED_LED, HIGH); delay(ms);
  digitalWrite(RED_LED, LOW);
}

// ===================================================
//                Wire Protocol (ESP-NOW)
// ===================================================

enum MsgKind : uint8_t {
  // Button -> Hub
  BTN_REGISTER = 1,
  BTN_PRESS    = 2,

  // Hub -> Button
  HUB_LED_SET   = 100,
  HUB_FEEDBACK  = 101,
  HUB_ALL_FLASH = 102,
};

enum FeedbackCode : uint8_t {
  FDBK_INCORRECT     = 0,
  FDBK_CORRECT       = 1,
  FDBK_ALREADYCOUNT  = 2,
  FDBK_LOCKEDOUT     = 3,
};

// Button -> Hub payloads
typedef struct __attribute__((packed)) {
  uint8_t kind;       // BTN_REGISTER
  uint8_t button_id;  // 0..63 (6 DIP switches)
  bool    ffa;        // FFA flag for this device
  uint8_t mac[6];     // device MAC (also available from recv_info)
} btn_register_t;

typedef struct __attribute__((packed)) {
  uint8_t  kind;       // BTN_PRESS
  uint8_t  button_id;  // 0..63
  bool     pressed;    // true on edge
  uint16_t press_id;   // increments per press (per-device)
  uint8_t  mac[6];     // sender MAC (convenience)
} btn_press_t;

// Hub -> Button payloads
typedef struct __attribute__((packed)) {
  uint8_t kind;  // HUB_LED_SET
  bool    on;
} hub_led_set_t;

typedef struct __attribute__((packed)) {
  uint8_t kind;    // HUB_FEEDBACK
  uint8_t result;  // FeedbackCode
} hub_feedback_t;

typedef struct __attribute__((packed)) {
  uint8_t  kind;   // HUB_ALL_FLASH
  uint8_t  times;
  uint16_t on_ms;
  uint16_t off_ms;
} hub_all_flash_t;

// ===================================================
//                Registration / Peers
// ===================================================

struct Peer {
  bool     present = false;
  uint8_t  mac[6]  = {0};
  uint8_t  button_id = 0;
  bool     ffa = false;

  // Runtime per round
  bool     counted = false;       // FFA: whether this MAC has already counted
  uint16_t last_press_id = 0;     // dedupe PRESS frames
};

static const int MAX_PEERS = 16;
static Peer peers[MAX_PEERS];

// Map button_id -> index into peers (for Sequence)
static const uint8_t MAX_BUTTON_ID = 63; // supports 6 DIP switches (values 0..63)
int idToIndex[MAX_BUTTON_ID + 1]; // -1 if unmapped

// Find peer slot by MAC (exact match)
int findPeerByMac(const uint8_t mac[6]) {
  for (int i = 0; i < MAX_PEERS; i++) {
    if (!peers[i].present) continue;
    if (memcmp(peers[i].mac, mac, 6) == 0) return i;
  }
  return -1;
}

// Reserve or return a free slot
int ensurePeerSlot(const uint8_t mac[6]) {
  int idx = findPeerByMac(mac);
  if (idx >= 0) return idx;
  for (int i = 0; i < MAX_PEERS; i++) {
    if (!peers[i].present) {
      peers[i].present = true;
      memcpy(peers[i].mac, mac, 6);
      peers[i].button_id = 0;
      peers[i].ffa = false;
      peers[i].counted = false;
      peers[i].last_press_id = 0;
      return i;
    }
  }
  return -1; // no space
}

// Clear all peers and mappings
void clearPeers() {
  for (int i = 0; i < MAX_PEERS; i++) {
    peers[i] = Peer{};
  }
  for (int i = 0; i <= MAX_BUTTON_ID; i++) idToIndex[i] = -1;
}

// Build id -> index map for sequence mode
void rebuildIdMap() {
  for (int i = 0; i <= MAX_BUTTON_ID; i++) idToIndex[i] = -1;
  for (int i = 0; i < MAX_PEERS; i++) {
    if (!peers[i].present) continue;
    if (peers[i].button_id > 0 && peers[i].button_id <= MAX_BUTTON_ID) {
      idToIndex[peers[i].button_id] = i; // last wins on conflict
    }
  }
}

// Count present devices
int countPresent() {
  int n = 0;
  for (int i = 0; i < MAX_PEERS; i++) if (peers[i].present) n++;
  return n;
}

// Check if all present devices have ffa=true
bool allFFA() {
  bool any = false;
  for (int i = 0; i < MAX_PEERS; i++) {
    if (!peers[i].present) continue;
    any = true;
    if (!peers[i].ffa) return false;
  }
  return any; // true only if at least one and all are ffa
}

// Reset per-round runtime fields
void resetRoundRuntime() {
  for (int i = 0; i < MAX_PEERS; i++) {
    if (!peers[i].present) continue;
    peers[i].counted = false;
    // Keep last_press_id to dedupe per device across round edges if desired.
  }
}

// ===================================================
//                 ESP-NOW helpers
// ===================================================

static esp_err_t sendTo(const uint8_t mac[6], const void* buf, size_t len) {
  return esp_now_send(mac, (const uint8_t*)buf, len);
}

void addEspNowPeerIfNeeded(const uint8_t mac[6]) {
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;
  p.encrypt = false;
  // It's OK if the peer already exists; esp_now_add_peer will fail, we ignore.
  esp_now_add_peer(&p);
}

// Unicast LED_SET
void sendLedSet(const uint8_t mac[6], bool on) {
  hub_led_set_t m{};
  m.kind = HUB_LED_SET;
  m.on   = on;
  addEspNowPeerIfNeeded(mac);
  sendTo(mac, &m, sizeof(m));
}

// Unicast FEEDBACK
void sendFeedback(const uint8_t mac[6], uint8_t code) {
  hub_feedback_t m{};
  m.kind = HUB_FEEDBACK;
  m.result = (uint8_t)code;
  addEspNowPeerIfNeeded(mac);
  sendTo(mac, &m, sizeof(m));
}

// Broadcast ALL_FLASH by iterating known peers
void sendAllFlash(uint8_t times = 5, uint16_t on_ms = 200, uint16_t off_ms = 200) {
  hub_all_flash_t m{};
  m.kind = HUB_ALL_FLASH;
  m.times = times;
  m.on_ms = on_ms;
  m.off_ms = off_ms;

  for (int i = 0; i < MAX_PEERS; i++) {
    if (!peers[i].present) continue;
    addEspNowPeerIfNeeded(peers[i].mac);
    sendTo(peers[i].mac, &m, sizeof(m));
  }
}

// Turn all LEDs OFF
void allLedsOff() {
  for (int i = 0; i < MAX_PEERS; i++) {
    if (!peers[i].present) continue;
    sendLedSet(peers[i].mac, false);
  }
}

// Turn all LEDs ON
void allLedsOn() {
  for (int i = 0; i < MAX_PEERS; i++) {
    if (!peers[i].present) continue;
    sendLedSet(peers[i].mac, true);
  }
}

// ===================================================
//                    Game State
// ===================================================

enum GameMode : uint8_t { MODE_IDLE=0, MODE_SEQUENCE=1, MODE_FFA=2 };

GameMode mode = MODE_IDLE;
uint8_t expected_id = 1;  // next expected id in sequence
int     participants = 0; // number of present devices this round
int     counted      = 0; // FFA: unique presses counted this round

// Registration window
bool            reg_open = false;
unsigned long   reg_deadline_ms = 0;
const uint32_t  REG_WINDOW_MS = 3000; // adjust as needed

void openRegistrationWindow() {
  reg_open = true;
  reg_deadline_ms = millis() + REG_WINDOW_MS;
  mode = MODE_IDLE;
  expected_id = 1;
  participants = 0;
  counted = 0;
  resetRoundRuntime();
  flashGreen(60);
  Serial.println("[Hub] Registration window OPEN");
}

void closeRegistrationAndStartRound() {
  reg_open = false;

  participants = countPresent();
  if (participants <= 0) {
    Serial.println("[Hub] No participants; staying idle.");
    mode = MODE_IDLE;
    return;
  }

  // Decide mode
  if (allFFA()) {
    mode = MODE_FFA;
    rebuildIdMap(); // not needed for FFA but keeps consistent
    resetRoundRuntime();
    counted = 0;

    Serial.print("[Hub] Starting FFA with "); Serial.print(participants); Serial.println(" participants.");
    // Per spec: LEDs ON for all; each valid press will turn its LED OFF
    allLedsOn();
  } else {
    mode = MODE_SEQUENCE;
    rebuildIdMap();
    expected_id = 1;

    Serial.print("[Hub] Starting SEQUENCE with "); Serial.print(participants); Serial.println(" participants.");
    // Turn all off, then light the first if it exists
    allLedsOff();
    int idx = (expected_id <= MAX_BUTTON_ID) ? idToIndex[expected_id] : -1;
    if (idx >= 0) {
      sendLedSet(peers[idx].mac, true);
    } else {
      Serial.println("[Hub] Warning: no device with button_id=1 is registered.");
    }
  }
}

// End the round, celebrate, and re-open registration
void finishRound() {
  Serial.println("[Hub] Round complete. Celebrating...");
  sendAllFlash(5, 200, 200);
  allLedsOff();
  mode = MODE_IDLE;
  expected_id = 1;
  counted = 0;
  // Immediately re-open registration for the next round while keeping peers
  openRegistrationWindow();
}

// ===================================================
//                 ESP-NOW callbacks
// ===================================================

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optional debug
  // Serial.print("Send status to "); for (int i=0;i<6;i++){Serial.printf("%02X",mac_addr[i]); if(i<5)Serial.print(":");}
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? " OK" : " FAIL");
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len < 1) return;
  const uint8_t kind = incomingData[0];

  // Source MAC from ESP-NOW
  const uint8_t* src = info->src_addr;

  switch (kind) {
    case BTN_REGISTER: {
      if (len < (int)sizeof(btn_register_t)) return;
      const btn_register_t* m = (const btn_register_t*)incomingData;

      // Start a fresh registration window if currently idle
      if (!reg_open && mode == MODE_IDLE) {
        openRegistrationWindow();
      }

      if (!reg_open) {
        // Ignore late registrations mid-round
        Serial.println("[Hub] Ignoring BTN_REGISTER (round in progress).");
        return;
      }

      int idx = ensurePeerSlot(src);
      if (idx < 0) {
        Serial.println("[Hub] Peer table full; registration dropped.");
        return;
      }

      peers[idx].button_id = m->button_id;
      peers[idx].ffa       = m->ffa;

      // Add as ESP-NOW peer to enable unicast
      addEspNowPeerIfNeeded(src);

      Serial.print("[Hub] REGISTER from ");
      for (int i=0;i<6;i++){ Serial.printf("%02X", src[i]); if(i<5)Serial.print(":"); }
      Serial.print("  id=");  Serial.print(m->button_id);
      Serial.print("  ffa="); Serial.println(m->ffa ? "YES" : "NO");

      // Extend the registration window slightly with each register (optional)
      reg_deadline_ms = millis() + REG_WINDOW_MS;
      break;
    }

    case BTN_PRESS: {
      if (len < (int)sizeof(btn_press_t)) return;
      const btn_press_t* m = (const btn_press_t*)incomingData;
      if (!m->pressed) return;

      // If we get a press while idle and no window is open, open a window (optional)
      if (mode == MODE_IDLE && !reg_open) {
        openRegistrationWindow();
        // Treat this press as part of the NEXT round; ignore now.
        Serial.println("[Hub] Press arrived; opened registration. Press ignored until round starts.");
        return;
      }

      // If still registering, ignore presses until we start the round.
      if (reg_open) {
        Serial.println("[Hub] Press received during registration window; ignored.");
        return;
      }

      int idx = findPeerByMac(src);
      if (idx < 0 || !peers[idx].present) {
        Serial.println("[Hub] Press from unknown MAC; ignored.");
        return;
      }

      // Dedupe by press_id
      if (m->press_id != 0 && m->press_id == peers[idx].last_press_id) {
        // Duplicate press frame → ignore
        return;
      }
      peers[idx].last_press_id = m->press_id;

      // Mode-specific handling
      if (mode == MODE_SEQUENCE) {
        uint8_t id = m->button_id;
        Serial.print("[Hub] SEQ press id="); Serial.print(id);
        Serial.print(" expected="); Serial.println(expected_id);

        if (id == expected_id) {
          // Correct press
          sendFeedback(src, FDBK_CORRECT);
          // Turn OFF current LED
          sendLedSet(src, false);
          expected_id++;

          // Advance or finish
          rebuildIdMap(); // ensure map is current if any hot-swap happened
          int nextIdx = (expected_id <= MAX_BUTTON_ID) ? idToIndex[expected_id] : -1;
          int total   = countPresent();

          if (expected_id <= MAX_BUTTON_ID && nextIdx >= 0) {
            // Light next
            sendLedSet(peers[nextIdx].mac, true);
          } else {
            // Completed sequence across however many IDs are mapped
            // We consider it done when expected_id > largest mapped or > total unique IDs used.
            finishRound();
          }
        } else {
          // Incorrect press
          sendFeedback(src, FDBK_INCORRECT);

          // Reset sequence: all OFF, then ID=1 ON if present
          allLedsOff();
          expected_id = 1;
          rebuildIdMap();
          int firstIdx = (expected_id <= MAX_BUTTON_ID) ? idToIndex[expected_id] : -1;
          if (firstIdx >= 0) sendLedSet(peers[firstIdx].mac, true);
        }

      } else if (mode == MODE_FFA) {
        Serial.print("[Hub] FFA press from MAC "); 
        for (int i=0;i<6;i++){ Serial.printf("%02X",src[i]); if(i<5)Serial.print(":"); }
        Serial.println();

        if (!peers[idx].counted) {
          // First valid press from this MAC
          peers[idx].counted = true;
          counted++;
          sendFeedback(src, FDBK_CORRECT);
          // Turn its LED OFF and effectively lock that device
          sendLedSet(src, false);

          if (counted >= participants) {
            // Everyone pressed exactly once
            finishRound();
          }
        } else {
          // Already counted -> anti-backdoor
          sendFeedback(src, FDBK_ALREADYCOUNT);
          // Keep LED OFF
          sendLedSet(src, false);
        }
      } else {
        // Unknown/idle mode
        sendFeedback(src, FDBK_INCORRECT);
      }
      break;
    }

    default:
      // Unknown kind
      break;
  }
}

// ===================================================
//                        Setup
// ===================================================
void setup() {
  Serial.begin(115200);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED,   OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED,   LOW);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[Hub] ESP-NOW init failed");
    while (true) { flashRed(200); delay(400); }
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  clearPeers();
  openRegistrationWindow();

  Serial.println("[Hub] Ready. Waiting for BTN_REGISTERs...");
}

// ===================================================
//                         Loop
// ===================================================
void loop() {
  // Close registration window and start the round
  if (reg_open && (long)(millis() - reg_deadline_ms) >= 0) {
    closeRegistrationAndStartRound();
  }

  // Nothing else to poll; all actions are event-driven
}