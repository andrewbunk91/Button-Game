// ================== BUTTON_V5.ino ==================
// Transport: ESP-NOW (station mode)
// Board: ESP32-WROOM-32
//
// Features:
// - Reads DIP (0..15) as button_id
// - Detects FFA mode when all 4 DIP switches are ON (value == 15)  <-- tweak if desired
// - On boot: sends REGISTER {button_id, ffa, mac[]}
// - On press: sends PRESS {button_id, pressed=true, press_id, mac[]}
// - Includes local press debounce
// - Receives Hub commands: LED_SET, FEEDBACK (Correct/Incorrect/AlreadyCounted/LockedOut), ALL_FLASH
// - In FFA mode: LED is ON at boot and turns OFF after first FEEDBACK; then the button locks until round end
//
// Pin map matches your current sketch:
//   BUTTON_PIN = 16 (to GND, uses INPUT_PULLUP -> active-low)
//   LED_PIN    = 27 (LED_ACTIVE_LOW controls polarity)
//   DIP pins   = {19,18,5,17}
//
// NOTE: The Hub must be updated to parse these new message "kinds" and fields.
// ===================================================

#include <esp_now.h>
#include <WiFi.h>

// ---------- LED polarity ----------
#define LED_ACTIVE_LOW false   // set true if your LED turns ON when pin is LOW

// ---------- IO pins ----------
const int BUTTON_PIN = 16;
const int LED_PIN    = 27;

// DIP pins (bit0..bit3)
constexpr uint8_t DIP_PINS[4] = {19, 18, 5, 17};
constexpr bool DIP_ACTIVE_LOW = true;  // using INPUT_PULLUP -> ON = LOW

// ---------- Hub MAC (update if needed) ----------
uint8_t hubAddress[] = {0xCC, 0xDB, 0xA7, 0x2D, 0xD2, 0x48};

// ---------- Debounce ----------
const unsigned long DEBOUNCE_MS = 30;
unsigned long lastDebounceTime = 0;
int lastReading = HIGH;     // INPUT_PULLUP idle
int debouncedState = HIGH;  // stable state

// ---------- Sequences / flash ----------
const int FLASH_COUNT = 5;
const unsigned long FLASH_ON_MS  = 200;
const unsigned long FLASH_OFF_MS = 200;

// ---------- Helpers ----------
inline void ledOn()  { digitalWrite(LED_PIN, LED_ACTIVE_LOW ? LOW  : HIGH); }
inline void ledOff() { digitalWrite(LED_PIN, LED_ACTIVE_LOW ? HIGH : LOW ); }

// ---------- Read STA MAC into array ----------
static void getStaMac(uint8_t out[6]) {
  esp_read_mac(out, ESP_MAC_WIFI_STA);
}

// ---------- Read DIP as 0..15 ----------
uint8_t readDip() {
  uint8_t v = 0;
  for (uint8_t i = 0; i < 4; i++) {
    int s = digitalRead(DIP_PINS[i]);
    if (DIP_ACTIVE_LOW) s = !s; // invert if ON=LOW
    if (s) v |= (1u << i);
  }
  return v;
}

// ---------- Button pressed edge (active-low) ----------
bool pressedEdge() {
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastReading) {
    lastDebounceTime = millis();
    lastReading = reading;
  }
  if ((millis() - lastDebounceTime) > DEBOUNCE_MS) {
    if (reading != debouncedState) {
      debouncedState = reading;
      if (debouncedState == LOW) return true; // just pressed
    }
  }
  return false;
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

// Button -> Hub
typedef struct __attribute__((packed)) {
  uint8_t kind;       // BTN_REGISTER
  uint8_t button_id;  // 0..15
  bool    ffa;        // true if this device is in FFA mode (DIP pattern)
  uint8_t mac[6];     // sender STA MAC (also available in recv_info)
} btn_register_t;

typedef struct __attribute__((packed)) {
  uint8_t  kind;       // BTN_PRESS
  uint8_t  button_id;  // 0..15
  bool     pressed;    // always true on edge
  uint16_t press_id;   // increments per press
  uint8_t  mac[6];     // sender MAC for convenience
} btn_press_t;

// Hub -> Button
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
//                 Button State
// ===================================================
static uint8_t deviceMac[6] = {0};
static uint8_t button_id = 0;
static bool    ffa_mode = false;
static bool    locked   = false;   // ignore further presses until next round (FFA lockout)
static uint16_t press_id = 0;

// ===================================================
//                 ESP-NOW Send helper
// ===================================================
static esp_err_t sendNow(const uint8_t* mac, const void* buf, size_t len) {
  return esp_now_send(mac, (const uint8_t*)buf, len);
}

// ===================================================
//                 Handle Hub Commands
// ===================================================
void handleHubPacket(const uint8_t* data, int len) {
  if (len < 1) return;
  uint8_t kind = data[0];

  switch (kind) {
    case HUB_LED_SET: {
      if (len < (int)sizeof(hub_led_set_t)) return;
      const hub_led_set_t* msg = (const hub_led_set_t*)data;
      if (msg->on) ledOn(); else ledOff();
      break;
    }
    case HUB_FEEDBACK: {
      if (len < (int)sizeof(hub_feedback_t)) return;
      const hub_feedback_t* msg = (const hub_feedback_t*)data;

      // Minimal UI: quick blink patterns on feedback if you want.
      // Also apply FFA lock and LED OFF on any terminal feedback.
      if (msg->result == FDBK_CORRECT) {
        // short "correct" blink (optional)
        for (int i = 0; i < 2; i++) { ledOn(); delay(80); ledOff(); delay(80); }
        // In FFA, turn LED off and lock
        if (ffa_mode) { ledOff(); locked = true; }
      } else if (msg->result == FDBK_INCORRECT) {
        // short "incorrect" blink (optional)
        for (int i = 0; i < 2; i++) { ledOn(); delay(40); ledOff(); delay(120); }
      } else if (msg->result == FDBK_ALREADYCOUNT || msg->result == FDBK_LOCKEDOUT) {
        // Ensure LED is off and lock in FFA
        if (ffa_mode) { ledOff(); locked = true; }
      }
      break;
    }
    case HUB_ALL_FLASH: {
      if (len < (int)sizeof(hub_all_flash_t)) return;
      const hub_all_flash_t* msg = (const hub_all_flash_t*)data;
      // End-of-round celebration
      for (uint8_t i = 0; i < msg->times; i++) {
        ledOn();  delay(msg->on_ms);
        ledOff(); delay(msg->off_ms);
      }
      // Round reset: clear lock; in non-FFA, LED remains under hub control.
      locked = false;
      if (ffa_mode) {
        // In FFA next round, LED should be ON until first valid press; either:
        //   - Hub will re-send LED_SET(on=true), OR
        //   - We locally turn it back on now. Choose ONE policy.
        // Here we choose local policy to honor "LED stays on until depressed".
        ledOn();
      }
      break;
    }
    default:
      // Unknown kind -> ignore
      break;
  }
}

// ===================================================
//                 ESP-NOW callbacks
// ===================================================
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optional: debug
  // Serial.print("Send status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  // Only accept from the hub (optional filter)
  if (memcmp(info->src_addr, hubAddress, 6) != 0) {
    // Unknown sender; ignore
    return;
  }
  handleHubPacket(incomingData, len);
}

// ===================================================
//                      Setup
// ===================================================
void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  ledOff();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  for (uint8_t p : DIP_PINS) pinMode(p, INPUT_PULLUP);

  // Compute initial ID and FFA
  button_id = readDip();
  ffa_mode  = (button_id == 15);   // <-- define FFA pattern here (all 4 switches ON)
  Serial.print("Button ID (DIP): "); Serial.println(button_id);
  Serial.print("FFA mode: ");       Serial.println(ffa_mode ? "YES" : "NO");

  // In FFA, LED should idle ON until depressed (per spec)
  if (ffa_mode) ledOn();

  // WiFi STA & ESP-NOW
  WiFi.mode(WIFI_STA);
  getStaMac(deviceMac);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  // Add Hub as peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, hubAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add hub as peer");
    return;
  }

  // Send REGISTER once
  btn_register_t reg{};
  reg.kind = BTN_REGISTER;
  reg.button_id = button_id;
  reg.ffa = ffa_mode;
  memcpy(reg.mac, deviceMac, 6);
  esp_err_t r = sendNow(hubAddress, &reg, sizeof(reg));
  Serial.println(r == ESP_OK ? "REGISTER sent" : "REGISTER send failed");
}

// ===================================================
//                       Loop
// ===================================================
void loop() {
  // Track DIP changes (optional live update)
  static uint8_t lastDip = 0xFF;
  uint8_t now = readDip();
  if (now != lastDip) {
    lastDip = now;
    button_id = now;
    bool newFFA = (button_id == 15); // keep FFA rule consistent with setup
    if (newFFA != ffa_mode) {
      ffa_mode = newFFA;
      locked = false;
      if (ffa_mode) ledOn(); else ledOff();
    }
    // Optionally inform hub about changed ID/FFA by re-sending REGISTER
    btn_register_t reg{};
    reg.kind = BTN_REGISTER;
    reg.button_id = button_id;
    reg.ffa = ffa_mode;
    memcpy(reg.mac, deviceMac, 6);
    sendNow(hubAddress, &reg, sizeof(reg));

    Serial.print("DIP changed -> ID=");
    Serial.print(button_id);
    Serial.print("  FFA=");
    Serial.println(ffa_mode ? "YES" : "NO");
  }

  // Handle button press
  if (pressedEdge()) {
    if (locked) {
      // Ignore presses while locked (FFA anti-backdoor)
      Serial.println("Press ignored (locked)");
      return;
    }

    btn_press_t msg{};
    msg.kind       = BTN_PRESS;
    msg.button_id  = button_id;
    msg.pressed    = true;
    msg.press_id   = ++press_id;
    memcpy(msg.mac, deviceMac, 6);

    esp_err_t s = sendNow(hubAddress, &msg, sizeof(msg));
    Serial.print("PRESS sent (id="); Serial.print(msg.press_id);
    Serial.println(s == ESP_OK ? ", ok)" : ", fail)");

    // In normal (sequence) mode we let the hub command LED changes.
    // In FFA, LED stays ON until hub acknowledges with FEEDBACK, then we lock.
  }
}
