// ================== BUTTON_V1.3.ino ==================
// Transport: ESP-NOW (station mode)
// Board: ESP32-WROOM-32
//
// Features:
// - Reads DIP (0..63) as button_id
// - Detects FFA mode when all DIP switches are ON (value == max)
// - On boot: sends REGISTER {button_id, ffa, mac[]}
// - On press: sends PRESS {button_id, pressed=true, press_id, mac[]}
// - Local press debounce
// - Receives Hub commands: LED_CONTROL, FEEDBACK, ROUND_RESET, ALL_FLASH
// - === NEW FEATURE: dynamic hub MAC === Learns the hub MAC from HUB_ANNOUNCE
//   packets and stores it in Preferences for automatic reconnects.
// - In sequence mode the hub can suspend hub-listening until the button is pressed
// - In FFA mode: device locks after first valid feedback until the next round
//
// Pin map:
//   BUTTON_PIN = 23 (to GND through switch, uses INPUT_PULLUP -> active-low)
//   LED_PIN    = 12 (drives the external indicator LED)
//   DIP pins   = {4,16,17,5,18,19} (switch 1..6)
//
// NOTE: The Hub must parse these message kinds/fields.
// ===================================================

#include <esp_now.h>    // Pull in the ESP-NOW networking helpers for peer-to-peer messages.
#include <WiFi.h>       // Give us access to Wi-Fi station mode so ESP-NOW can run.
#include <Preferences.h>  // === NEW FEATURE: dynamic hub MAC === Persist the learned hub MAC between boots.
#include <cstdio>       // === NEW FEATURE: dynamic hub MAC === Provide snprintf for MAC formatting in logs.
// If the low-level ESP-IDF Wi-Fi header exists on this platform...
#include <esp_wifi.h>          // ...include it so we can directly query the station MAC address.
#define HAVE_ESP_WIFI 1        // Remember that esp_wifi.h was available for later checks.

#if defined(__has_include)  // Check whether the compiler supports __has_include before using it.
#  if __has_include(<esp_idf_version.h>)  // Some cores expose the ESP-IDF version header as well.
#    include <esp_idf_version.h>         // Include it so we can adapt callbacks to IDF v5 changes.
#  endif
#endif

const int ESPNOW_CHANNEL = 1; // can be 1–13, but both devices must match

// ---------- Button polarity ----------
#define BUTTON_ACTIVE_LOW true   // Treat a LOW reading as pressed because the switch pulls to ground.

// LED polarity (set to true if LED turns on when driven LOW)
#define LED_ACTIVE_LOW true

// ---------- IO pins ----------
const int BUTTON_PIN = 23;  // Physical pin that the arcade button is wired to.
const int LED_PIN    = 12;  // GPIO driving the indicator LED supplied on the button harness.

// DIP pins (bit0..bit5)
constexpr uint8_t DIP_PINS[] = {4, 16, 17, 5, 18, 19};                   // Each DIP switch leg arrives on one of these pins.
constexpr uint8_t DIP_COUNT = sizeof(DIP_PINS) / sizeof(DIP_PINS[0]);    // Work out how many DIP pins we declared.
constexpr bool DIP_ACTIVE_LOW = true;                                    // INPUT_PULLUP means an ON switch reads LOW.
constexpr uint8_t FFA_PATTERN = (1u << DIP_COUNT) - 1;                   // Bitmask where all DIP switches are ON (value 63).

// === NEW FEATURE: dynamic hub MAC === Persistent storage for the hub MAC so
// buttons can follow whichever hub last announced itself.
static Preferences prefs;                 // Handle to the ESP32 Preferences API.
static bool prefsReady = false;           // Tracks whether prefs.begin() succeeded.
static uint8_t hubAddress[6] = {0};       // Learned hub MAC (all zeros until discovered).
static bool hubMacKnown = false;          // Whether we currently know the hub MAC.
static const char* PREF_NAMESPACE = "button"; // Namespace for storing configuration.
static const char* PREF_KEY_HUBMAC = "hubmac"; // Key under which we persist the MAC bytes.

// === NEW FEATURE: dynamic hub MAC === Format MAC addresses for logging.
static String formatMac(const uint8_t mac[6]) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

// === NEW FEATURE: dynamic hub MAC === Load the hub MAC from NVS if available.
static bool loadStoredHubMac() {
  if (!prefsReady) return false;
  size_t len = prefs.getBytesLength(PREF_KEY_HUBMAC);
  if (len != 6) return false;
  prefs.getBytes(PREF_KEY_HUBMAC, hubAddress, sizeof(hubAddress));
  hubMacKnown = true;
  return true;
}

// === NEW FEATURE: dynamic hub MAC === Persist the current hub MAC to NVS.
static void storeHubMac(const uint8_t mac[6]) {
  if (!prefsReady) return;
  prefs.putBytes(PREF_KEY_HUBMAC, mac, 6);
}

// ---------- LED helpers ----------
inline void ledOn()  { digitalWrite(LED_PIN, LED_ACTIVE_LOW ? LOW  : HIGH); }
inline void ledOff() { digitalWrite(LED_PIN, LED_ACTIVE_LOW ? HIGH : LOW ); }

static void blinkErrorIndicator(uint8_t times = 3, uint16_t on_ms = 120, uint16_t off_ms = 120) {
  for (uint8_t i = 0; i < times; i++) {
    ledOn();
    delay(on_ms);
    ledOff();
    delay(off_ms);
  }
}

// ---------- Debounce & press throttle ----------
const unsigned long DEBOUNCE_MS = 30;                        // Wait this many ms before trusting a changed reading.
unsigned long lastDebounceTime = 0;                           // Timestamp of the last raw change we saw on the button pin.
const int BUTTON_PRESSED_LEVEL = BUTTON_ACTIVE_LOW ? LOW : HIGH;  // Translate "pressed" into the electrical level we expect.
const int BUTTON_IDLE_LEVEL    = BUTTON_ACTIVE_LOW ? HIGH : LOW;  // Translate "not pressed" into the electrical level we expect.
int lastReading = BUTTON_IDLE_LEVEL;                          // Remember the most recent raw reading.
int debouncedState = BUTTON_IDLE_LEVEL;                       // Remember the last stable (debounced) state.
unsigned long lastPressTxMs = 0;                              // When we last transmitted a press message to the hub.
const unsigned long PRESS_COOLDOWN_MS = 150;                  // Minimum gap between transmissions to prevent double sends.

// ---------- Read STA MAC into array ----------
static void getStaMac(uint8_t out[6]) {               // Helper that fetches the Wi-Fi MAC into the provided buffer.
#if defined(ESP_PLATFORM) && defined(HAVE_ESP_WIFI)   // If we are on ESP-IDF with esp_wifi.h available...
  if (esp_wifi_get_mac(WIFI_IF_STA, out) == ESP_OK) { // ...ask the Wi-Fi driver directly for the station MAC.
    return;                                           // If that worked we are done.
  }
#endif  // End of direct esp_wifi.h path.
  WiFi.macAddress(out);                               // Fallback: use the Arduino WiFi helper to read the MAC.
}  // End of getStaMac helper.

// ---------- Read DIP as 0..63 ----------
uint8_t readDip() {                          // Convert the 6 DIP switches into a numeric ID.
  uint8_t v = 0;                              // Start with all bits cleared.
  for (uint8_t i = 0; i < DIP_COUNT; i++) {   // Walk across each DIP switch.
    int s = digitalRead(DIP_PINS[i]);         // Read the electrical state of this switch leg.
    if (DIP_ACTIVE_LOW) s = !s;              // Flip the meaning if ON corresponds to LOW.
    if (s) v |= (1u << i);                   // Set the bit position for this switch if it is ON.
  }
  return v;                                   // Hand the combined number back to the caller.
}  // End of readDip helper.

// ---------- Button pressed edge (active-low) ----------
bool pressedEdge() {                                // Report true exactly once whenever the button is pressed.
  int reading = digitalRead(BUTTON_PIN);            // Grab the current electrical state from the input pin.
  if (reading != lastReading) {                     // If the raw value changed since last time...
    lastDebounceTime = millis();                    // ...note when the change happened...
    lastReading = reading;                          // ...and remember this new raw value.
  }
  if ((millis() - lastDebounceTime) > DEBOUNCE_MS) {        // Once the value has been stable long enough...
    if (reading != debouncedState) {                        // ...and it differs from our last stable state...
      debouncedState = reading;                             // ...treat this new level as the debounced state.
      if (debouncedState == BUTTON_PRESSED_LEVEL) return true; // When the new stable state equals "pressed", flag the press.
    }
  }
  return false;                                     // Otherwise no fresh press event yet.
}  // End of pressedEdge helper.

// ===================================================
//                Wire Protocol (ESP-NOW)
// ===================================================

enum MsgKind : uint8_t {          // All the message types that travel between hub and button.
  // Button -> Hub
  BTN_REGISTER    = 1,            // Button announces itself and shares its ID.
  BTN_PRESS       = 2,            // Button reports that it was pressed.

  // Hub -> Button
  HUB_LED_CONTROL = 100,          // Hub commands LED and listening behaviour.
  HUB_FEEDBACK    = 101,          // Hub tells the button whether the press counted.
  HUB_ROUND_RESET = 102,          // Hub instructs the button to unlock for a new round.
  HUB_ALL_FLASH   = 103,          // Hub asks every button to flash for celebration.
  // === NEW FEATURE: dynamic hub MAC === Broadcast that carries the hub's MAC
  // so the button can pair itself without hardcoding the address.
  HUB_ANNOUNCE    = 200,
};  // End of MsgKind definitions.

enum FeedbackCode : uint8_t {     // Possible verdicts sent with HUB_FEEDBACK.
  FDBK_INCORRECT     = 0,         // The press did not match what the hub wanted.
  FDBK_CORRECT       = 1,         // The press was good and counted.
  FDBK_ALREADYCOUNT  = 2,         // The hub already counted this button for the round.
  FDBK_LOCKEDOUT     = 3,         // The hub refuses input until the next reset.
};  // End of FeedbackCode definitions.

// Button -> Hub
typedef struct __attribute__((packed)) {  // Layout for the registration message.
  uint8_t kind;       // BTN_REGISTER
  uint8_t button_id;  // 0..63
  bool    ffa;        // true if this device is in FFA mode (DIP pattern)
  uint8_t mac[6];     // sender STA MAC (also available in recv_info)
} btn_register_t;  // Struct describing the BTN_REGISTER packet.

typedef struct __attribute__((packed)) {  // Layout for the press message.
  uint8_t  kind;       // BTN_PRESS
  uint8_t  button_id;  // 0..63
  bool     pressed;    // always true on edge
  uint16_t press_id;   // increments per press
  uint8_t  mac[6];     // sender MAC for convenience
} btn_press_t;     // Struct describing the BTN_PRESS packet.

typedef struct __attribute__((packed)) {  // Layout for hub LED control.
  uint8_t kind;       // HUB_LED_CONTROL
  bool    on;         // Desired LED state
  bool    suspend_rx; // When true, suspend processing of hub packets until button pressed
} hub_led_control_t;

typedef struct __attribute__((packed)) {  // Layout for hub feedback.
  uint8_t kind;    // HUB_FEEDBACK
  uint8_t result;  // FeedbackCode
} hub_feedback_t;  // Struct describing the HUB_FEEDBACK packet.

typedef struct __attribute__((packed)) {  // Layout for hub round reset.
  uint8_t kind;   // HUB_ROUND_RESET
} hub_round_reset_t;  // Struct describing the HUB_ROUND_RESET packet.

typedef struct __attribute__((packed)) {
  uint8_t  kind;   // HUB_ALL_FLASH
  uint8_t  times;  // Number of flash cycles
  uint16_t on_ms;  // Duration LED should stay ON per cycle
  uint16_t off_ms; // Duration LED should stay OFF per cycle
} hub_all_flash_t;

// NOTE: Keep all ESP-NOW wire protocol structs grouped here so Arduino's
// autogenerated prototypes see the types before any functions that use them.
// This ordering avoids "hub_announce_t does not name a type" build errors.
// === NEW FEATURE: dynamic hub MAC === Structure describing HUB_ANNOUNCE.
struct hub_announce_t {
  uint8_t kind;   // HUB_ANNOUNCE
  uint8_t mac[6]; // Hub MAC address payload
} __attribute__((packed));

typedef struct hub_announce_t hub_announce_t;

// ===================================================
//                 Button State
// ===================================================
static uint8_t deviceMac[6] = {0};  // Storage for the MAC address of this button board.
static uint8_t button_id = 0;       // Current ID read from the DIP switches.
static bool    ffa_mode = false;    // Whether we detected the "all switches on" pattern.
static bool    locked   = false;    // Ignore further presses until next round (FFA lockout or hub lockout).
static uint16_t press_id = 0;       // Counter that lets the hub ignore duplicate press packets.
static bool    listeningSuspended = false;  // When true, ignore most hub commands until the button is pressed.

// ===================================================
//                 ESP-NOW Send helper
// ===================================================
static esp_err_t sendNow(const uint8_t* mac, const void* buf, size_t len) {  // Small wrapper to send bytes over ESP-NOW.
  return esp_now_send(mac, (const uint8_t*)buf, len);                        // Forward the call to the ESP-NOW stack.
}  // End of sendNow helper.

// === NEW FEATURE: dynamic hub MAC === Make sure the hub is registered as an
// ESP-NOW peer before we try to send packets.
static bool ensureHubPeer(bool *added = nullptr) {
  if (!hubMacKnown) return false;
  if (esp_now_is_peer_exist(hubAddress)) {
    if (added) *added = false;
    return true;
  }
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, hubAddress, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;
  esp_err_t status = esp_now_add_peer(&peerInfo);
  if (status == ESP_OK) {
    if (added) *added = true;
    return true;
  }
  if (status == ESP_ERR_ESPNOW_EXIST) {
    if (added) *added = false;
    return true;
  }
  Serial.println("[Button] Failed to add hub as peer");
  Serial.printf("ESP-NOW error: %d\n", status);
  return false;
}

static bool sendRegister() {                            // Craft and transmit the registration message to the hub.
  if (!hubMacKnown) {                                   // === NEW FEATURE: dynamic hub MAC === Guard against unknown hubs.
    Serial.println("[Button] Cannot send REGISTER (hub MAC unknown).");
    return false;
  }
  if (!ensureHubPeer()) {                               // Make sure ESP-NOW knows how to reach the hub.
    return false;
  }
  btn_register_t reg{};                                 // Start with a zeroed struct so no garbage leaves the board.
  reg.kind = BTN_REGISTER;                              // Tag the message as a registration.
  reg.button_id = button_id;                            // Include the DIP-derived ID.
  reg.ffa = ffa_mode;                                   // Tell the hub whether we are in free-for-all mode.
  memcpy(reg.mac, deviceMac, 6);                        // Copy our MAC for the hub's records.
  return sendNow(hubAddress, &reg, sizeof(reg)) == ESP_OK;  // Send the packet and report success to the caller.
}  // End of sendRegister helper.

// === NEW FEATURE: dynamic hub MAC === React to HUB_ANNOUNCE broadcasts by
// updating our stored MAC and registering with the announcing hub.
static void handleHubAnnounce(const hub_announce_t* msg) {
  bool hadMac = hubMacKnown;
  bool changed = (!hubMacKnown) || (memcmp(hubAddress, msg->mac, 6) != 0);
  uint8_t oldMac[6] = {0};
  if (hadMac) memcpy(oldMac, hubAddress, 6);

  if (changed) {
    if (hadMac) {
      esp_now_del_peer(oldMac);
      Serial.print("[Button] Replacing stored hub MAC with ");
      Serial.println(formatMac(msg->mac));
    } else {
      Serial.print("[Button] Learned hub MAC ");
      Serial.println(formatMac(msg->mac));
    }
    memcpy(hubAddress, msg->mac, 6);
    hubMacKnown = true;
    storeHubMac(hubAddress);
  } else {
    Serial.print("[Button] HUB_ANNOUNCE matches stored MAC ");
    Serial.println(formatMac(hubAddress));
  }

  bool peerAdded = false;
  if (!ensureHubPeer(&peerAdded)) return;

  if (!hadMac || changed || peerAdded) {
    Serial.println(sendRegister() ? "REGISTER sent" : "REGISTER send failed");
  }
}

// ===================================================
//                 Handle Hub Commands
// ===================================================
void handleHubPacket(const uint8_t* data, int len) {            // React to packets coming from the hub.
  if (len < 1) return;                                          // Ignore empty packets that do not have a kind byte.
  uint8_t kind = data[0];                                       // The first byte always tells us what type of message it is.

  if (listeningSuspended && kind != HUB_ROUND_RESET) {          // When suspended, ignore everything except hard resets.
    return;
  }

  switch (kind) {                                               // Branch based on the message kind.
    case HUB_LED_CONTROL: {                                     // Hub is commanding our LED state.
      if (len < (int)sizeof(hub_led_control_t)) return;         // Verify payload size.
      const hub_led_control_t* msg = (const hub_led_control_t*)data;
      if (msg->on) ledOn(); else ledOff();                      // Apply LED state immediately.
      listeningSuspended = msg->suspend_rx;                     // Optionally suspend hub message handling.
      locked = false;                                           // Clear any lock so we can report the press.
      break;
    }

    case HUB_FEEDBACK: {                                        // Hub is telling us if a press counted.
      if (len < (int)sizeof(hub_feedback_t)) return;            // Make sure the packet is long enough to read safely.
      const hub_feedback_t* msg = (const hub_feedback_t*)data;  // Reinterpret the bytes as a feedback structure.
      uint8_t result = msg->result;                             // Pull out the feedback code for easier reading.
      if (result == FDBK_CORRECT) {                             // The hub accepted our press.
        if (ffa_mode) {                                         // In free-for-all mode...
          ledOff();                                             // ...turn the LED off to show the success.
          locked = true;                                        // ...lock out further presses until a reset.
        }
        Serial.println("[Button] Feedback: CORRECT");           // Log what happened for troubleshooting.
      } else if (result == FDBK_INCORRECT) {                    // The hub rejected the press.
        Serial.println("[Button] Feedback: INCORRECT");          // Log that the press was wrong.
      } else if (result == FDBK_ALREADYCOUNT) {                 // The hub says we already scored this round.
        if (ffa_mode) {                                         // FFA devices should stop sending after that.
          ledOff();                                             // Ensure LED goes dark as we retire for the round.
          locked = true;                                        // Stop responding until a reset message arrives.
        }
        Serial.println("[Button] Feedback: ALREADY COUNTED");    // Log that this MAC already scored.
      } else if (result == FDBK_LOCKEDOUT) {                    // The hub explicitly told us to stop for now.
        locked = true;                                          // Enforce the lockout immediately.
        ledOff();                                               // Make sure the LED is off while locked.
        Serial.println("[Button] Feedback: LOCKED OUT");         // Log that the hub locked us out.
      }
      break;                                                    // Done processing HUB_FEEDBACK.
    }

    case HUB_ROUND_RESET: {                                     // Hub is opening a fresh round.
      if (len < (int)sizeof(hub_round_reset_t)) return;         // Sanity-check the size even though the struct is tiny.
      locked = false;                                           // Unlock the button so the next press can go through.
      listeningSuspended = false;                               // Resume accepting hub commands.
      Serial.println("[Button] Round reset received.");         // Let the log show that we are ready again.
      if (ffa_mode) ledOn(); else ledOff();                     // Restore idle LED state appropriate to the mode.
      break;                                                    // Done handling reset.
    }

    case HUB_ALL_FLASH: {                                       // Hub wants us to flash the LED repeatedly.
      if (len < (int)sizeof(hub_all_flash_t)) return;           // Ensure we can read the structure safely.
      const hub_all_flash_t* msg = (const hub_all_flash_t*)data;
      for (uint8_t i = 0; i < msg->times; i++) {                // Flash according to hub timing.
        ledOn();  delay(msg->on_ms);
        ledOff(); delay(msg->off_ms);
      }
      locked = true;                                            // Stay locked until the next hub reset.
      listeningSuspended = false;                               // Allow commands once the next round begins.
      break;
    }

    default:
      break;                                                    // Silently discard messages we do not understand.
  }
}  // End of handleHubPacket.

// ===================================================
//                 ESP-NOW callbacks
// ===================================================
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {  // ESP-IDF v5 style send callback.
  (void)info;                                                                // We are not using the detailed info struct.
  // Optional: debug
  // Serial.print("Send status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}  // End of onDataSent (IDF v5 variant).
#else
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {     // Older core callback signature.
  (void)mac_addr;                                                            // We are not inspecting which peer it was.
  // Optional: debug
  // Serial.print("Send status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}  // End of onDataSent (legacy variant).
#endif  // End of callback signature selection.

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {  // Called whenever we get ESP-NOW data.
  if (len < 1) return;                                                     // Need at least the kind byte.
  uint8_t kind = incomingData[0];                                          // Extract the message type.

  if (kind == HUB_ANNOUNCE) {                                              // === NEW FEATURE: dynamic hub MAC ===
    if (len < (int)sizeof(hub_announce_t)) return;                         // Validate size before casting.
    const hub_announce_t* msg = (const hub_announce_t*)incomingData;       // Interpret payload as HUB_ANNOUNCE.
    handleHubAnnounce(msg);                                                // Update stored MAC and auto-register.
    return;                                                                // Nothing else to do for announcements.
  }

  if (!hubMacKnown) {                                                      // Ignore all other traffic until we know the hub.
    return;
  }

  if (memcmp(info->src_addr, hubAddress, 6) != 0) {                        // Drop packets that are not from our hub.
    return;
  }

  handleHubPacket(incomingData, len);                                      // Otherwise hand the data to our parser.
}  // End of onDataRecv callback.

// ===================================================
//                      Setup
// ===================================================
void setup() {                                                          // Arduino setup runs once on boot.
  Serial.begin(115200);                                                 // Open the USB serial port for logs.

  pinMode(LED_PIN, OUTPUT);                                             // Drive the LED from this GPIO.
  ledOff();                                                             // Start with the LED dark until the hub says otherwise.

  pinMode(BUTTON_PIN, INPUT_PULLUP);                                    // Configure the button pin so it rests HIGH and goes LOW when pressed.
  for (uint8_t p : DIP_PINS) pinMode(p, INPUT_PULLUP);                  // Give each DIP switch an internal pull-up as well.

  // Compute initial ID and FFA
  button_id = readDip();                                                // Read the switches to decide our button number.
  ffa_mode  = (button_id == FFA_PATTERN);                               // Check whether the switches indicate "all ON" for FFA.
  Serial.print("Button ID (DIP): "); Serial.println(button_id);        // Log the ID for sanity checking.
  Serial.print("FFA mode: ");       Serial.println(ffa_mode ? "YES" : "NO");  // Log whether we believe we are in FFA.

  if (ffa_mode) {                                                       // In free-for-all the LED idles ON until a valid press.
    ledOn();
  }

  // === NEW FEATURE: dynamic hub MAC === Open preferences early so incoming
  // HUB_ANNOUNCE packets can be saved immediately.
  prefsReady = prefs.begin(PREF_NAMESPACE, false);
  if (!prefsReady) {
    Serial.println("[Button] Preferences init failed; hub MAC will not persist.");
    blinkErrorIndicator();
  } else if (loadStoredHubMac()) {
    Serial.print("[Button] Loaded stored hub MAC ");
    Serial.println(formatMac(hubAddress));
  } else {
    Serial.println("[Button] No stored hub MAC; waiting for HUB_ANNOUNCE.");
  }

  // WiFi STA & ESP-NOW
  WiFi.mode(WIFI_STA);                                                  // Put the ESP32 into station mode (required for ESP-NOW).
  esp_err_t chanStatus = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (chanStatus != ESP_OK) {
    Serial.println("[Button] Failed to set ESP-NOW channel.");
    Serial.printf("ESP-NOW error: %d\n", chanStatus);
  }
  getStaMac(deviceMac);                                                 // Capture our MAC so we can share it with the hub.

  esp_err_t initStatus = esp_now_init();                                 // Start the ESP-NOW subsystem and confirm success.
  if (initStatus != ESP_OK) {
    Serial.println("ESP-NOW init failed");                             // Report an error if initialization failed.
    Serial.printf("ESP-NOW error: %d\n", initStatus);
    return;                                                             // Bail out because nothing else will work without ESP-NOW.
  }

  esp_now_register_send_cb(onDataSent);                                 // Ask ESP-NOW to call us after each send attempt.
  esp_now_register_recv_cb(onDataRecv);                                 // Ask ESP-NOW to call us whenever data arrives.
  if (hubMacKnown) {                                                    // Reconnect automatically if we already know the hub.
    if (ensureHubPeer()) {
      Serial.println(sendRegister() ? "REGISTER sent" : "REGISTER send failed");
    } else {
      Serial.println("[Button] Failed to add stored hub as peer; waiting for HUB_ANNOUNCE.");
    }
  }
}  // End of setup.

// ===================================================
//                       Loop
// ===================================================
void loop() {                                                         // Arduino loop runs repeatedly after setup.
  // Handle button press
  if (pressedEdge()) {                                                // Only act when a fresh press is detected.
    if (locked) {                                                     // If the hub told us to wait...
      // Ignore presses while locked (FFA anti-backdoor)
      Serial.println("Press ignored (locked)");                       // ...explain why the press was skipped.
      return;                                                         // ...and leave the loop early.
    }

    unsigned long nowMs = millis();                                   // Record the current time in milliseconds.
    if (lastPressTxMs != 0 && (nowMs - lastPressTxMs) < PRESS_COOLDOWN_MS) {  // Ensure we are not spamming messages.
      return;                                                         // If we sent too recently, ignore this bounce.
    }

    if (!ffa_mode) {                                                  // Sequence buttons darken themselves immediately.
      ledOff();
    }
    listeningSuspended = false;                                       // Re-open hub listening for the next instruction.

    btn_press_t msg{};                                                // Prepare a clean press message.
    msg.kind       = BTN_PRESS;                                       // Identify the message type.
    msg.button_id  = button_id;                                       // Include our DIP ID so the hub knows who we are.
    msg.pressed    = true;                                            // Mark that this packet represents a press event.
    msg.press_id   = ++press_id;                                      // Increment the press counter and include it.
    memcpy(msg.mac, deviceMac, 6);                                    // Attach our MAC for any extra filtering.

    esp_err_t s = sendNow(hubAddress, &msg, sizeof(msg));             // Ship the packet to the hub and capture the status.
    lastPressTxMs = nowMs;                                            // Remember when we sent it for cooldown logic.
    Serial.print("PRESS sent (id="); Serial.print(msg.press_id);      // Log the press number...
    Serial.println(s == ESP_OK ? ", ok)" : ", fail)");             // ...and whether the transmission succeeded.

    // Feedback from the hub will manage locking/unlocking via HUB_FEEDBACK/HUB_ROUND_RESET.
  }
}  // End of loop.
