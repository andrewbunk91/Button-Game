// ================== HUB_V16_RELAY_TFT.ino ==================
// Transport: ESP-NOW (station mode)
// Board: ESP32-WROOM-32
//
// Same as HUB_V10_OPTO, but drives a RELAY instead of an LED or optocoupler.
// The relay input pin is wired:
//   ESP32 GPIO -> relay IN (active HIGH)  (VCC and GND shared)
//
// When a round finishes, the relay energizes for ~2 seconds.
// ======================================================

#include <esp_now.h>
#include <WiFi.h>
#include <esp_idf_version.h>
#include <cstdio>

// ---------- Display Setup ----------
#define USER_SETUP_LOADED
#define ST7789_DRIVER
#define TFT_WIDTH  135
#define TFT_HEIGHT 240
#define TFT_OFFSET_X 52
#define TFT_OFFSET_Y 40
#define TFT_MISO   -1
#define TFT_MOSI   19
#define TFT_SCLK   18
#define TFT_CS     5
#define TFT_DC     16
#define TFT_RST    23
#define TFT_BL     4
#define TFT_BACKLIGHT_ON HIGH
#define LOAD_GLCD
#define SPI_FREQUENCY 40000000

#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();
bool screenReady = false;
int screenCursorX = 0;
int screenCursorY = 0;
int screenLineHeight = 0;
bool screenNeedsClear = false;

void resetScreenCursor() {
  screenCursorX = 0;
  screenCursorY = 0;
  screenNeedsClear = false;
  if (screenReady) tft.setCursor(0, 0);
}

void clearScreen() {
  if (!screenReady) return;
  tft.fillScreen(TFT_BLACK);
  resetScreenCursor();
}

void ensureCursorInBounds() {
  if (!screenReady) return;
  screenCursorX = tft.getCursorX();
  screenCursorY = tft.getCursorY();
  if (screenCursorY + screenLineHeight > TFT_HEIGHT) {
    screenNeedsClear = true;
    screenCursorX = 0;
    screenCursorY = 0;
  }
}

// ---------- printScreen Helper ----------
void printScreen(const String &msg, bool newline = true) {
  Serial.print(msg);
  if (newline) Serial.println();
  if (!screenReady) return;

  if (screenNeedsClear) clearScreen();

  int start = 0;
  int len = msg.length();
  while (start < len) {
    if (screenNeedsClear) clearScreen();
    int idx = msg.indexOf('\n', start);
    int end = (idx < 0) ? len : idx;
    String segment = msg.substring(start, end);
    tft.setCursor(screenCursorX, screenCursorY);
    tft.print(segment);
    ensureCursorInBounds();
    if (idx >= 0) {
      tft.println();
      ensureCursorInBounds();
      start = idx + 1;
    } else {
      start = len;
    }
  }
  if (newline) {
    if (!screenNeedsClear) {
      tft.println();
      ensureCursorInBounds();
    } else {
      screenCursorX = 0;
      screenCursorY = 0;
    }
  } else {
    screenCursorX = tft.getCursorX();
    screenCursorY = tft.getCursorY();
  }
}

String formatMac(const uint8_t mac[6]) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

enum ResetEvent : uint8_t {
  RESET_EVENT_NONE = 0,
  RESET_EVENT_TAP  = 1,
  RESET_EVENT_HOLD = 2,
};

// ---------- Relay output (replaces optocoupler) ----------
#define RELAY_PIN 12  // GPIO driving the relay module

static inline void flashGreen(uint16_t ms = 30) {
  // HIGH = energize relay briefly
  digitalWrite(RELAY_PIN, HIGH);
  delay(ms);
  digitalWrite(RELAY_PIN, LOW);
}

// ===================================================
//                Wire Protocol (ESP-NOW)
// ===================================================

enum MsgKind : uint8_t {
  BTN_REGISTER = 1,
  BTN_PRESS    = 2,
  HUB_FEEDBACK    = 101,
  HUB_ROUND_RESET = 102,
};

enum FeedbackCode : uint8_t {
  FDBK_INCORRECT     = 0,
  FDBK_CORRECT       = 1,
  FDBK_ALREADYCOUNT  = 2,
  FDBK_LOCKEDOUT     = 3,
};

typedef struct __attribute__((packed)) {
  uint8_t kind;
  uint8_t button_id;
  bool    ffa;
  uint8_t mac[6];
} btn_register_t;

typedef struct __attribute__((packed)) {
  uint8_t  kind;
  uint8_t  button_id;
  bool     pressed;
  uint16_t press_id;
  uint8_t  mac[6];
} btn_press_t;

typedef struct __attribute__((packed)) {
  uint8_t kind;
  uint8_t result;
} hub_feedback_t;

typedef struct __attribute__((packed)) {
  uint8_t kind;
} hub_round_reset_t;

// ===================================================
//                Registration / Peers
// ===================================================

struct Peer {
  bool     present = false;
  uint8_t  mac[6]  = {0};
  uint8_t  button_id = 0;
  bool     ffa = false;
  bool     counted = false;
  uint16_t last_press_id = 0;
};

static const int MAX_PEERS = 16;
static Peer peers[MAX_PEERS];
static const uint8_t MAX_BUTTON_ID = 63;
int idToIndex[MAX_BUTTON_ID + 1];

int findPeerByMac(const uint8_t mac[6]) {
  for (int i = 0; i < MAX_PEERS; i++) {
    if (!peers[i].present) continue;
    if (memcmp(peers[i].mac, mac, 6) == 0) return i;
  }
  return -1;
}

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
  return -1;
}

void clearPeers() {
  for (int i = 0; i < MAX_PEERS; i++) peers[i] = Peer{};
  for (int i = 0; i <= MAX_BUTTON_ID; i++) idToIndex[i] = -1;
}

void rebuildIdMap() {
  for (int i = 0; i <= MAX_BUTTON_ID; i++) idToIndex[i] = -1;
  for (int i = 0; i < MAX_PEERS; i++) {
    if (!peers[i].present) continue;
    if (peers[i].button_id > 0 && peers[i].button_id <= MAX_BUTTON_ID)
      idToIndex[peers[i].button_id] = i;
  }
}

int countPresent() {
  int n = 0;
  for (int i = 0; i < MAX_PEERS; i++) if (peers[i].present) n++;
  return n;
}

bool allFFA() {
  bool any = false;
  for (int i = 0; i < MAX_PEERS; i++) {
    if (!peers[i].present) continue;
    any = true;
    if (!peers[i].ffa) return false;
  }
  return any;
}

void resetRoundRuntime() {
  for (int i = 0; i < MAX_PEERS; i++) {
    if (!peers[i].present) continue;
    peers[i].counted = false;
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
  esp_now_add_peer(&p);
}

void sendFeedback(const uint8_t mac[6], uint8_t code) {
  hub_feedback_t m{};
  m.kind = HUB_FEEDBACK;
  m.result = (uint8_t)code;
  addEspNowPeerIfNeeded(mac);
  sendTo(mac, &m, sizeof(m));
}

void broadcastRoundReset() {
  hub_round_reset_t m{};
  m.kind = HUB_ROUND_RESET;
  for (int i = 0; i < MAX_PEERS; i++) {
    if (!peers[i].present) continue;
    addEspNowPeerIfNeeded(peers[i].mac);
    sendTo(peers[i].mac, &m, sizeof(m));
  }
}

// ===================================================
//                    Game State
// ===================================================

enum GameMode : uint8_t { MODE_IDLE=0, MODE_SEQUENCE=1, MODE_FFA=2 };

GameMode mode = MODE_IDLE;
uint8_t expected_id = 1;
int participants = 0;
int counted = 0;

bool reg_open = false;
unsigned long reg_deadline_ms = 0;
const uint32_t REG_WINDOW_MS = 3000;

bool locked_out = false;

const int RESET_BUTTON_PIN = 18;
const bool RESET_ACTIVE_LOW = true;
const unsigned long RESET_DEBOUNCE_MS = 40;
const int RESET_PRESSED_LEVEL  = RESET_ACTIVE_LOW ? LOW  : HIGH;
const int RESET_RELEASED_LEVEL = RESET_ACTIVE_LOW ? HIGH : LOW;
int resetLastReading = RESET_RELEASED_LEVEL;
int resetStableState = RESET_RELEASED_LEVEL;
unsigned long resetLastDebounceMs = 0;
const unsigned long RESET_HOLD_THRESHOLD_MS = 1000;
bool resetPressActive = false;
bool resetHoldFired = false;
bool resetHoldActive = false;
unsigned long resetPressStartMs = 0;

void openRegistrationWindow() {
  reg_open = true;
  reg_deadline_ms = millis() + REG_WINDOW_MS;
  mode = MODE_IDLE;
  expected_id = 1;
  participants = 0;
  counted = 0;
  resetRoundRuntime();
  flashGreen(60);
  printScreen("[Hub] Registration window OPEN");
}

bool startRoundFromPeers() {
  reg_open = false;
  participants = countPresent();
  if (participants <= 0) {
    printScreen("[Hub] No participants; staying idle.");
    mode = MODE_IDLE;
    return false;
  }
  resetRoundRuntime();
  counted = 0;
  expected_id = 1;
  if (allFFA()) {
    mode = MODE_FFA;
    rebuildIdMap();
    printScreen("[Hub] Starting FFA with ", false);
    printScreen(String(participants), false);
    printScreen(" participants.");
  } else {
    mode = MODE_SEQUENCE;
    rebuildIdMap();
    printScreen("[Hub] Starting SEQUENCE with ", false);
    printScreen(String(participants), false);
    printScreen(" participants.");
    int idx = (expected_id <= MAX_BUTTON_ID) ? idToIndex[expected_id] : -1;
    if (idx < 0) printScreen("[Hub] Warning: no device with button_id=1 is registered.");
  }
  broadcastRoundReset();
  return true;
}

void closeRegistrationAndStartRound() { startRoundFromPeers(); }

void finishRound() {
  printScreen("[Hub] Round complete. Awaiting reset...");
  broadcastRoundReset();
  mode = MODE_IDLE;
  expected_id = 1;
  counted = 0;
  resetRoundRuntime();
  reg_open = false;
  locked_out = true;

  // steady relay ON for 2 s to indicate round end
  digitalWrite(RELAY_PIN, HIGH);
  delay(2000);
  digitalWrite(RELAY_PIN, LOW);
}

// ===================================================
// Reset button polling
// ===================================================

ResetEvent pollResetButton() {
  int reading = digitalRead(RESET_BUTTON_PIN);
  if (reading != resetLastReading) {
    resetLastDebounceMs = millis();
    resetLastReading = reading;
  }
  if ((millis() - resetLastDebounceMs) > RESET_DEBOUNCE_MS) {
    if (reading != resetStableState) {
      resetStableState = reading;
      if (resetStableState == RESET_PRESSED_LEVEL) {
        resetPressActive = true;
        resetHoldFired = false;
        resetHoldActive = false;
        resetPressStartMs = millis();
      } else {
        if (resetPressActive) {
          unsigned long heldMs = millis() - resetPressStartMs;
          bool firedHold = resetHoldFired;
          resetPressActive = false;
          resetHoldActive = false;
          resetHoldFired = false;
          if (!firedHold && heldMs < RESET_HOLD_THRESHOLD_MS)
            return RESET_EVENT_TAP;
        }
      }
    }
  }
  if (resetPressActive && !resetHoldFired) {
    if ((millis() - resetPressStartMs) >= RESET_HOLD_THRESHOLD_MS) {
      resetHoldFired = true;
      resetHoldActive = true;
      return RESET_EVENT_HOLD;
    }
  }
  return RESET_EVENT_NONE;
}

void handleShortResetPress() {
  printScreen("[Hub] Reset button tapped.");
  locked_out = false;
  expected_id = 1;
  counted = 0;
  resetRoundRuntime();
  broadcastRoundReset();
  if (!startRoundFromPeers()) openRegistrationWindow();
}

void handleResetHold() {
  printScreen("[Hub] Reset button held; reopening registration.");
  locked_out = false;
  expected_id = 1;
  counted = 0;
  resetRoundRuntime();
  broadcastRoundReset();
  mode = MODE_IDLE;
  reg_open = true;
  reg_deadline_ms = millis() + REG_WINDOW_MS;
  participants = countPresent();
  rebuildIdMap();
}

// ===================================================
// ESP-NOW callbacks
// ===================================================

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) { }

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len < 1) return;
  const uint8_t kind = incomingData[0];
  const uint8_t* src = info->src_addr;
  switch (kind) {
    case BTN_REGISTER: {
      if (len < (int)sizeof(btn_register_t)) return;
      const btn_register_t* m = (const btn_register_t*)incomingData;
      if (!reg_open && mode == MODE_IDLE) openRegistrationWindow();
      if (!reg_open) {
        printScreen("[Hub] Ignoring BTN_REGISTER (round in progress).");
        return;
      }
      int idx = ensurePeerSlot(src);
      if (idx < 0) {
        printScreen("[Hub] Peer table full; registration dropped.");
        return;
      }
      peers[idx].button_id = m->button_id;
      peers[idx].ffa = m->ffa;
      addEspNowPeerIfNeeded(src);
      String log = String("[Hub] REGISTER from ") + formatMac(src) + "  id=" + String(m->button_id) +
                   "  ffa=" + (m->ffa ? "YES" : "NO");
      printScreen(log);
      reg_deadline_ms = millis() + REG_WINDOW_MS;
      break;
    }

    case BTN_PRESS: {
      if (len < (int)sizeof(btn_press_t)) return;
      const btn_press_t* m = (const btn_press_t*)incomingData;
      if (!m->pressed) return;
      if (locked_out) {
        printScreen("[Hub] Press ignored (round complete, awaiting reset).");
        sendFeedback(src, FDBK_LOCKEDOUT);
        return;
      }
      if (mode == MODE_IDLE && !reg_open) {
        openRegistrationWindow();
        printScreen("[Hub] Press arrived; opened registration. Ignored until round starts.");
        return;
      }
      if (reg_open) {
        printScreen("[Hub] Press during registration; ignored.");
        return;
      }
      int idx = findPeerByMac(src);
      if (idx < 0 || !peers[idx].present) {
        printScreen("[Hub] Press from unknown MAC; ignored.");
        return;
      }
      if (m->press_id != 0 && m->press_id == peers[idx].last_press_id) return;
      peers[idx].last_press_id = m->press_id;
      if (mode == MODE_SEQUENCE) {
        uint8_t id = m->button_id;
        printScreen("[Hub] SEQ press id=", false);
        printScreen(String(id), false);
        printScreen(" expected=", false);
        printScreen(String(expected_id));
        if (id == expected_id) {
          sendFeedback(src, FDBK_CORRECT);
          expected_id++;
          rebuildIdMap();
          int nextIdx = (expected_id <= MAX_BUTTON_ID) ? idToIndex[expected_id] : -1;
          if (nextIdx < 0) finishRound();
        } else sendFeedback(src, FDBK_INCORRECT);
      } else if (mode == MODE_FFA) {
        printScreen("[Hub] FFA press from MAC ", false);
        printScreen(formatMac(src));
        if (!peers[idx].counted) {
          peers[idx].counted = true;
          counted++;
          sendFeedback(src, FDBK_CORRECT);
          if (counted >= participants) finishRound();
        } else sendFeedback(src, FDBK_ALREADYCOUNT);
      } else sendFeedback(src, FDBK_INCORRECT);
      break;
    }
    default: break;
  }
}

// ===================================================
// Setup / Loop
// ===================================================

void setup() {
  Serial.begin(115200);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, TFT_BACKLIGHT_ON);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextWrap(true);
  screenLineHeight = tft.fontHeight();
  if (screenLineHeight <= 0) screenLineHeight = 16;
  screenReady = true;
  resetScreenCursor();
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // relay off
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    printScreen("[Hub] ESP-NOW init failed");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  clearPeers();
  openRegistrationWindow();
  printScreen("[Hub] Ready. Waiting for BTN_REGISTERs...");
}

void loop() {
  if (reg_open && (long)(millis() - reg_deadline_ms) >= 0)
    closeRegistrationAndStartRound();

  ResetEvent resetEvent = pollResetButton();
  if (resetEvent == RESET_EVENT_TAP) handleShortResetPress();
  else if (resetEvent == RESET_EVENT_HOLD) handleResetHold();

  if (resetHoldActive && reg_open)
    reg_deadline_ms = millis() + REG_WINDOW_MS;
}
