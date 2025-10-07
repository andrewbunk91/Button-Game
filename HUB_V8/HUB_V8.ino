// ================== HUB_V8.ino ==================
// Transport: ESP-NOW (station mode)
// Board: ESP32-WROOM-32
//
// Features:
// - Receives BTN_REGISTER (button_id, ffa, mac[]) from each button on boot
// - Closes a short registration window automatically, then starts a round
// - If ALL registered buttons have ffa=true -> FFA mode
//   * First valid press per MAC counts; then lockout until round reset
// - Else -> Sequence mode
//   * Tracks expected button ID; ignores incorrect presses without resetting
// - Sends Hub->Button commands: HUB_FEEDBACK, HUB_ROUND_RESET
// - Tracks per-MAC last_press_id to ignore duplicates
// - Simple end-of-round reset and re-open registration for the next round
//
// The single green LED on the hub board is kept for optional local feedback.
// ===================================================

#include <esp_now.h>  // Provide ESP-NOW peer-to-peer networking primitives.
#include <WiFi.h>     // Allow us to enter Wi-Fi station mode required by ESP-NOW.
#include <esp_idf_version.h>  // Provide ESP-IDF version macros for compatibility checks.

// ---------- Optional local debug LEDs ----------
#define GREEN_LED 12  // GPIO pin wired to the green debug LED on the hub board.

static inline void flashGreen(uint16_t ms = 120) {   // Briefly pulse the green LED for human feedback.
  digitalWrite(GREEN_LED, HIGH); delay(ms);          // Turn the LED on, wait the requested time.
  digitalWrite(GREEN_LED, LOW);                      // Turn the LED back off.
}  // End of flashGreen helper.

// ===================================================
//                Wire Protocol (ESP-NOW)
// ===================================================

enum MsgKind : uint8_t {          // All message types exchanged with the buttons.
  // Button -> Hub
  BTN_REGISTER = 1,               // Button announces its ID and whether it wants FFA.
  BTN_PRESS    = 2,               // Button reports that someone pressed it.

  // Hub -> Button
  HUB_FEEDBACK    = 101,          // Hub replies letting the button know if the press counted.
  HUB_ROUND_RESET = 102,          // Hub instructs every button to unlock for a new round.
};  // End of MsgKind definitions.

enum FeedbackCode : uint8_t {     // Possible feedback verdicts from the hub.
  FDBK_INCORRECT     = 0,         // The press did not match the expected button.
  FDBK_CORRECT       = 1,         // The press was correct and counted.
  FDBK_ALREADYCOUNT  = 2,         // The hub already counted this button during the round.
  FDBK_LOCKEDOUT     = 3,         // The hub is ignoring this button until the next reset.
};  // End of FeedbackCode definitions.

// Button -> Hub payloads
typedef struct __attribute__((packed)) {  // Description of a registration message.
  uint8_t kind;       // BTN_REGISTER
  uint8_t button_id;  // 0..63 (6 DIP switches)
  bool    ffa;        // FFA flag for this device
  uint8_t mac[6];     // device MAC (also available from recv_info)
} btn_register_t;     // Struct describing the BTN_REGISTER packet.

typedef struct __attribute__((packed)) {  // Description of a press message.
  uint8_t  kind;       // BTN_PRESS
  uint8_t  button_id;  // 0..63
  bool     pressed;    // true on edge
  uint16_t press_id;   // increments per press (per-device)
  uint8_t  mac[6];     // sender MAC (convenience)
} btn_press_t;         // Struct describing the BTN_PRESS packet.

typedef struct __attribute__((packed)) {  // Description of a feedback message.
  uint8_t kind;    // HUB_FEEDBACK
  uint8_t result;  // FeedbackCode
} hub_feedback_t;  // Struct describing the HUB_FEEDBACK packet.

typedef struct __attribute__((packed)) {  // Description of the round reset message.
  uint8_t kind;   // HUB_ROUND_RESET
} hub_round_reset_t;  // Struct describing the HUB_ROUND_RESET packet.

// ===================================================
//                Registration / Peers
// ===================================================

struct Peer {                                      // Everything the hub tracks about one button peer.
  bool     present = false;                        // Whether this slot is currently assigned to a peer.
  uint8_t  mac[6]  = {0};                          // The ESP-NOW MAC address for the peer.
  uint8_t  button_id = 0;                          // DIP-derived button ID reported during registration.
  bool     ffa = false;                            // Whether the peer prefers free-for-all scoring.

  // Runtime per round
  bool     counted = false;                        // FFA: remember if this MAC already counted this round.
  uint16_t last_press_id = 0;                      // Highest press_id seen to filter duplicates.
};  // End of Peer definition.

static const int MAX_PEERS = 16;                   // Upper bound on how many buttons can register at once.
static Peer peers[MAX_PEERS];                      // Storage array for peer records.

// Map button_id -> index into peers (for Sequence)
static const uint8_t MAX_BUTTON_ID = 63;           // Supports DIP switch values 0..63.
int idToIndex[MAX_BUTTON_ID + 1];                  // Lookup table: button_id -> peers[] index or -1.

// Find peer slot by MAC (exact match)
int findPeerByMac(const uint8_t mac[6]) {          // Search for a peer slot using the MAC address.
  for (int i = 0; i < MAX_PEERS; i++) {            // Check each slot in the peers array.
    if (!peers[i].present) continue;               // Skip unused slots quickly.
    if (memcmp(peers[i].mac, mac, 6) == 0) return i;  // Return the index if the MAC matches exactly.
  }
  return -1;                                       // Not found.
}  // End of findPeerByMac.

// Reserve or return a free slot
int ensurePeerSlot(const uint8_t mac[6]) {         // Make sure a peer record exists for this MAC.
  int idx = findPeerByMac(mac);                    // Try to find an existing slot first.
  if (idx >= 0) return idx;                        // Reuse it if it already exists.
  for (int i = 0; i < MAX_PEERS; i++) {            // Otherwise look for a free slot.
    if (!peers[i].present) {                       // Found an unused entry.
      peers[i].present = true;                     // Mark the slot as active.
      memcpy(peers[i].mac, mac, 6);                // Store the MAC address for later lookups.
      peers[i].button_id = 0;                      // Clear previous button ID until registration arrives.
      peers[i].ffa = false;                        // Default to sequence mode until told otherwise.
      peers[i].counted = false;                    // Reset round state.
      peers[i].last_press_id = 0;                  // Reset duplicate filter.
      return i;                                    // Hand back the slot index to the caller.
    }
  }
  return -1; // no space
}  // End of ensurePeerSlot.

// Clear all peers and mappings
void clearPeers() {                                // Wipe every peer slot and ID mapping.
  for (int i = 0; i < MAX_PEERS; i++) {            // Walk across each peer entry.
    peers[i] = Peer{};                             // Reset the struct to its default state.
  }
  for (int i = 0; i <= MAX_BUTTON_ID; i++) idToIndex[i] = -1;  // Reset ID lookup table to "unassigned".
}  // End of clearPeers.

// Build id -> index map for sequence mode
void rebuildIdMap() {                              // Refresh the button_id -> peer index cache.
  for (int i = 0; i <= MAX_BUTTON_ID; i++) idToIndex[i] = -1;  // Start by clearing all mappings.
  for (int i = 0; i < MAX_PEERS; i++) {            // Examine each peer slot.
    if (!peers[i].present) continue;               // Skip empty slots.
    if (peers[i].button_id > 0 && peers[i].button_id <= MAX_BUTTON_ID) {  // Only map valid IDs (non-zero).
      idToIndex[peers[i].button_id] = i;           // Remember which peer sits at this button ID.
    }
  }
}  // End of rebuildIdMap.

// Count present devices
int countPresent() {                               // Return how many peers are currently registered.
  int n = 0;                                       // Start a counter.
  for (int i = 0; i < MAX_PEERS; i++) if (peers[i].present) n++;  // Increment for every active slot.
  return n;                                        // Give the total back to the caller.
}  // End of countPresent.

// Check if all present devices have ffa=true
bool allFFA() {                                    // Decide whether every registered peer wants FFA mode.
  bool any = false;                                // Track whether at least one peer exists.
  for (int i = 0; i < MAX_PEERS; i++) {            // Inspect each peer slot.
    if (!peers[i].present) continue;               // Skip unused entries.
    any = true;                                    // Remember that we saw at least one peer.
    if (!peers[i].ffa) return false;               // If any peer is not FFA, the answer is false.
  }
  return any; // true only if at least one and all are ffa
}  // End of allFFA.

// Reset per-round runtime fields
void resetRoundRuntime() {                         // Clear counters that are specific to the current round.
  for (int i = 0; i < MAX_PEERS; i++) {            // Touch each peer slot.
    if (!peers[i].present) continue;               // Ignore empty slots.
    peers[i].counted = false;                      // Allow the peer to score again in FFA mode.
    // Keep last_press_id to dedupe per device across round edges if desired.
  }
}  // End of resetRoundRuntime.

// ===================================================
//                 ESP-NOW helpers
// ===================================================

static esp_err_t sendTo(const uint8_t mac[6], const void* buf, size_t len) {  // Wrapper for sending bytes over ESP-NOW.
  return esp_now_send(mac, (const uint8_t*)buf, len);                        // Defer to the ESP-NOW stack.
}  // End of sendTo helper.

void addEspNowPeerIfNeeded(const uint8_t mac[6]) {   // Ensure the ESP-NOW stack knows how to reach this MAC.
  esp_now_peer_info_t p{};                           // Start with a zeroed peer descriptor.
  memcpy(p.peer_addr, mac, 6);                       // Copy the MAC into the descriptor.
  p.channel = 0;                                     // Use the default Wi-Fi channel (must match both sides).
  p.encrypt = false;                                 // We do not use encryption for these packets.
  // It's OK if the peer already exists; esp_now_add_peer will fail, we ignore.
  esp_now_add_peer(&p);                              // Ask ESP-NOW to add the peer (ignore duplicate errors).
}  // End of addEspNowPeerIfNeeded.

// Unicast FEEDBACK
void sendFeedback(const uint8_t mac[6], uint8_t code) {  // Send a HUB_FEEDBACK verdict to one button.
  hub_feedback_t m{};                                    // Zero the message struct before filling it.
  m.kind = HUB_FEEDBACK;                                 // Identify the packet type.
  m.result = (uint8_t)code;                              // Attach the requested feedback code.
  addEspNowPeerIfNeeded(mac);                            // Make sure the peer exists in the ESP-NOW table.
  sendTo(mac, &m, sizeof(m));                            // Ship the packet out.
}  // End of sendFeedback.

void broadcastRoundReset() {                                  // Send a HUB_ROUND_RESET to every registered peer.
  hub_round_reset_t m{};                                      // Prepare a tiny reset packet.
  m.kind = HUB_ROUND_RESET;                                   // Identify the packet so buttons unlock.

  for (int i = 0; i < MAX_PEERS; i++) {                       // Walk through every peer slot.
    if (!peers[i].present) continue;                          // Skip empty entries.
    addEspNowPeerIfNeeded(peers[i].mac);                      // Ensure the peer exists in the ESP-NOW table.
    sendTo(peers[i].mac, &m, sizeof(m));                      // Transmit the reset packet.
  }
}  // End of broadcastRoundReset.

// ===================================================
//                    Game State
// ===================================================

enum GameMode : uint8_t { MODE_IDLE=0, MODE_SEQUENCE=1, MODE_FFA=2 };  // Three possible hub states.

GameMode mode = MODE_IDLE;                     // Current mode (idle until enough peers register).
uint8_t expected_id = 1;                      // Next button ID we are waiting for in sequence mode.
int     participants = 0;                     // Number of peers participating in the current round.
int     counted      = 0;                     // Number of unique FFA presses that already counted.

// Registration window
bool            reg_open = false;             // Whether the hub is currently accepting registrations.
unsigned long   reg_deadline_ms = 0;          // Millisecond timestamp for when registration should close.
const uint32_t  REG_WINDOW_MS = 3000;         // Duration of the registration window in milliseconds.

const int RESET_BUTTON_PIN = 18;              // GPIO attached to the manual reset button.
const bool RESET_ACTIVE_LOW = true;           // The button pulls the pin low when pressed.
const unsigned long RESET_DEBOUNCE_MS = 40;   // Debounce interval for the reset button.
const int RESET_PRESSED_LEVEL  = RESET_ACTIVE_LOW ? LOW  : HIGH;  // Electrical level meaning "pressed".
const int RESET_RELEASED_LEVEL = RESET_ACTIVE_LOW ? HIGH : LOW;   // Electrical level meaning "released".
int resetLastReading = RESET_RELEASED_LEVEL;  // Last raw reading observed on the reset pin.
int resetStableState = RESET_RELEASED_LEVEL;  // Last debounced state for the reset pin.
unsigned long resetLastDebounceMs = 0;        // Timestamp of the last raw change for debouncing.

void openRegistrationWindow() {                             // Begin a new registration period.
  reg_open = true;                                          // Allow buttons to register again.
  reg_deadline_ms = millis() + REG_WINDOW_MS;               // Set the deadline for closing registration.
  mode = MODE_IDLE;                                        // Reset the game mode to idle.
  expected_id = 1;                                         // Start sequence tracking from button 1.
  participants = 0;                                        // Clear participant count.
  counted = 0;                                             // Reset FFA scoring count.
  resetRoundRuntime();                                     // Clear per-peer round state.
  flashGreen(60);                                          // Flash the green LED for quick feedback.
  Serial.println("[Hub] Registration window OPEN");        // Log the state change for the operator.
}  // End of openRegistrationWindow.

bool startRoundFromPeers() {                                // Attempt to start a new round using registered peers.
  reg_open = false;                                         // Close the registration window.

  participants = countPresent();                            // Count how many peers we have.
  if (participants <= 0) {                                  // If nobody is around...
    Serial.println("[Hub] No participants; staying idle."); // ...log and stay idle.
    mode = MODE_IDLE;                                       // Explicitly remain in idle mode.
    return false;                                           // Indicate that the round did not start.
  }

  resetRoundRuntime();                                      // Make sure per-round counters are clear.
  counted = 0;                                              // Reset the FFA counted presses tracker.
  expected_id = 1;                                          // Start expecting button 1 for sequence mode.

  // Decide mode
  if (allFFA()) {                                           // If everyone asked for free-for-all...
    mode = MODE_FFA;                                        // ...select FFA mode.
    rebuildIdMap();                                         // Refresh mapping so diagnostics stay accurate.

    Serial.print("[Hub] Starting FFA with "); Serial.print(participants); Serial.println(" participants.");  // Log FFA kickoff details.
  } else {                                                  // Otherwise use sequence mode.
    mode = MODE_SEQUENCE;                                   // Switch into sequence mode.
    rebuildIdMap();                                         // Build the ID lookup table for quick presses.

    Serial.print("[Hub] Starting SEQUENCE with "); Serial.print(participants); Serial.println(" participants.");  // Log sequence kickoff details.
    int idx = (expected_id <= MAX_BUTTON_ID) ? idToIndex[expected_id] : -1;  // Check that button 1 exists.
    if (idx < 0) {                                          // Warn if nobody registered as button 1.
      Serial.println("[Hub] Warning: no device with button_id=1 is registered.");  // Alert operator about missing ID 1.
    }
  }

  broadcastRoundReset();                                   // Tell all peers to unlock for the new round.
  return true;                                              // Signal that a round started.
}  // End of startRoundFromPeers.

void closeRegistrationAndStartRound() {            // Helper that closes registration and tries to start a round.
  startRoundFromPeers();                           // Reuse the main round start logic.
}  // End of closeRegistrationAndStartRound.

// End the round and wait for manual reset
void finishRound() {                               // Prepare the system to wait for the next manual reset.
  Serial.println("[Hub] Round complete. Awaiting reset...");  // Let the operator know the round ended.
  broadcastRoundReset();                           // Tell buttons to unlock (in case some stayed locked).
  mode = MODE_IDLE;                                // Return to idle mode.
  expected_id = 1;                                 // Reset the expected sequence ID.
  counted = 0;                                     // Clear FFA counters.
  resetRoundRuntime();                             // Wipe per-peer round flags.
  reg_open = false;                                // Keep registration closed until the reset button is pressed.
  digitalWrite(GREEN_LED, HIGH);                   // Light the green LED steadily to show the round is done.
  delay(2000);                                     // Keep the LED illuminated for two seconds as requested.
  digitalWrite(GREEN_LED, LOW);                    // Turn the LED back off afterward.
}  // End of finishRound.

bool resetPressedEdge() {                          // Detect a clean press of the manual reset button.
  int reading = digitalRead(RESET_BUTTON_PIN);     // Grab the current electrical level.
  if (reading != resetLastReading) {               // If the raw value changed...
    resetLastDebounceMs = millis();                // ...record when it happened...
    resetLastReading = reading;                    // ...and remember the new raw value.
  }

  if ((millis() - resetLastDebounceMs) > RESET_DEBOUNCE_MS) {  // After the value is stable long enough...
    if (reading != resetStableState) {                         // ...and it differs from the last stable state...
      resetStableState = reading;                              // ...update the debounced state.
      if (resetStableState == RESET_PRESSED_LEVEL) {           // If the new stable state is "pressed"...
        return true;                                           // ...report the button press edge.
      }
    }
  }

  return false;                                   // Otherwise no new reset press yet.
}  // End of resetPressedEdge.

void handleResetButtonPress() {                    // Respond to a confirmed press of the reset button.
  Serial.println("[Hub] Reset button pressed.");  // Log the manual reset action.
  expected_id = 1;                                 // Always restart the sequence at button 1.
  counted = 0;                                     // Clear the FFA counted presses.
  resetRoundRuntime();                             // Reset per-peer round state.

  broadcastRoundReset();                           // Tell all buttons to unlock immediately.

  if (!startRoundFromPeers()) {                    // Try to launch a round with existing registrations.
    openRegistrationWindow();                      // If nobody is around, reopen registration.
  }
}  // End of handleResetButtonPress.

// ===================================================
//                 ESP-NOW callbacks
// ===================================================

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {   // Optional send callback for debugging.
  // Optional debug
  // Serial.print("Send status to "); for (int i=0;i<6;i++){Serial.printf("%02X",info->des_addr[i]); if(i<5)Serial.print(":");}
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? " OK" : " FAIL");
}  // End of onDataSent.

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {  // Handle inbound ESP-NOW packets.
  if (len < 1) return;                                   // Drop empty packets with no kind byte.
  const uint8_t kind = incomingData[0];                  // First byte describes the message type.

  // Source MAC from ESP-NOW
  const uint8_t* src = info->src_addr;                   // Remember who sent the packet.

  switch (kind) {                                        // Branch based on message type.
    case BTN_REGISTER: {                                 // Button is registering its ID.
      if (len < (int)sizeof(btn_register_t)) return;     // Ensure the payload is large enough to read.
      const btn_register_t* m = (const btn_register_t*)incomingData;  // Interpret bytes as a registration.

      // Start a fresh registration window if currently idle
      if (!reg_open && mode == MODE_IDLE) {              // If we were idle with registration closed...
        openRegistrationWindow();                        // ...re-open so this and others can join.
      }

      if (!reg_open) {                                   // If registration is closed...
        Serial.println("[Hub] Ignoring BTN_REGISTER (round in progress).");  // ...explain why we ignore it.
        return;                                          // Do not change anything mid-round.
      }

      int idx = ensurePeerSlot(src);                     // Reserve a slot for this MAC.
      if (idx < 0) {                                     // If we ran out of space...
        Serial.println("[Hub] Peer table full; registration dropped.");  // ...log the issue...
        return;                                          // ...and abort processing.
      }

      peers[idx].button_id = m->button_id;               // Record the reported button ID.
      peers[idx].ffa       = m->ffa;                     // Record whether the button wants FFA.

      // Add as ESP-NOW peer to enable unicast
      addEspNowPeerIfNeeded(src);                        // Make sure we can send feedback directly.

      Serial.print("[Hub] REGISTER from ");              // Print the MAC address in hex.
      for (int i=0;i<6;i++){ Serial.printf("%02X", src[i]); if(i<5)Serial.print(":"); }  // Print MAC bytes separated by colons.
      Serial.print("  id=");  Serial.print(m->button_id);                                  // Show the reported button ID.
      Serial.print("  ffa="); Serial.println(m->ffa ? "YES" : "NO");                      // Show the FFA flag value.

      // Extend the registration window slightly with each register (optional)
      reg_deadline_ms = millis() + REG_WINDOW_MS;        // Allow a little more time for others to join.
      break;                                             // Done handling registration.
    }

    case BTN_PRESS: {                                    // Button reports a press event.
      if (len < (int)sizeof(btn_press_t)) return;        // Ensure the payload fits the struct.
      const btn_press_t* m = (const btn_press_t*)incomingData;  // Interpret bytes as a press message.
      if (!m->pressed) return;                           // Ignore releases; we only care about the press edge.

      // If we get a press while idle and no window is open, open a window (optional)
      if (mode == MODE_IDLE && !reg_open) {              // If the hub was idle...
        openRegistrationWindow();                        // ...open registration so devices can join.
        // Treat this press as part of the NEXT round; ignore now.
        Serial.println("[Hub] Press arrived; opened registration. Press ignored until round starts.");  // Explain the decision.
        return;                                          // Ignore the press until the round is started.
      }

      // If still registering, ignore presses until we start the round.
      if (reg_open) {                                    // During registration we ignore gameplay input.
        Serial.println("[Hub] Press received during registration window; ignored.");          // Remind operator about the ignore.
        return;                                          // Leave without starting the round yet.
      }

      int idx = findPeerByMac(src);                      // Locate which peer sent the press.
      if (idx < 0 || !peers[idx].present) {              // If the sender is unknown...
        Serial.println("[Hub] Press from unknown MAC; ignored.");                             // Log that we discarded the packet.
        return;                                          // ...ignore the packet.
      }

      // Dedupe by press_id
      if (m->press_id != 0 && m->press_id == peers[idx].last_press_id) {  // If we already saw this press ID...
        // Duplicate press frame → ignore
        return;                                          // ...drop the duplicate.
      }
      peers[idx].last_press_id = m->press_id;            // Remember this press ID for future dedupe.

      // Mode-specific handling
      if (mode == MODE_SEQUENCE) {                     // Sequence mode expects presses in numerical order.
        uint8_t id = m->button_id;                     // Capture which button claimed the press.
        Serial.print("[Hub] SEQ press id="); Serial.print(id);                             // Log which ID was pressed.
        Serial.print(" expected="); Serial.println(expected_id);                           // Log which ID we expected.

        if (id == expected_id) {                       // Press matches the expected ID.
          // Correct press
          sendFeedback(src, FDBK_CORRECT);             // Congratulate the button.
          expected_id++;                               // Move on to the next ID in the sequence.

          // Advance or finish
          rebuildIdMap(); // ensure map is current if any hot-swap happened
          int nextIdx = (expected_id <= MAX_BUTTON_ID) ? idToIndex[expected_id] : -1;  // Check if next ID is present.

          if (nextIdx < 0) {                           // No peer registered with the next ID.
            // Completed sequence across however many IDs are mapped
            finishRound();                             // Treat this as the end of the round.
          }
        } else {                                       // Wrong button pressed.
          // Incorrect press: notify but keep waiting for the same expected_id
          sendFeedback(src, FDBK_INCORRECT);           // Tell the button it was incorrect but keep waiting.
        }

      } else if (mode == MODE_FFA) {                   // Free-for-all mode counts one press per MAC.
        Serial.print("[Hub] FFA press from MAC ");                                          // Announce which MAC just pressed.
        for (int i=0;i<6;i++){ Serial.printf("%02X",src[i]); if(i<5)Serial.print(":"); }   // Print MAC bytes separated by colons.
        Serial.println();                                                                   // Finish the log line cleanly.

        if (!peers[idx].counted) {                     // First valid press from this MAC this round.
          // First valid press from this MAC
          peers[idx].counted = true;                   // Remember that this MAC already scored.
          counted++;                                   // Increase the number of counted players.
          sendFeedback(src, FDBK_CORRECT);             // Confirm the press to the button.

          if (counted >= participants) {               // Everyone who registered has now pressed once.
            // Everyone pressed exactly once
            finishRound();                             // End the round automatically.
          }
        } else {                                       // This MAC already scored.
          // Already counted -> anti-backdoor
          sendFeedback(src, FDBK_ALREADYCOUNT);        // Remind the button it already counted.
        }
      } else {                                        // If we somehow get here while idle...
        // Unknown/idle mode
        sendFeedback(src, FDBK_INCORRECT);             // ...treat the press as incorrect.
      }
      break;                                          // Done handling BTN_PRESS.
    }

    default:
      // Unknown kind
      break;                                          // Ignore unrecognized message types.
  }
}  // End of onDataRecv.

// ===================================================
//                        Setup
// ===================================================
void setup() {                                                      // Runs once at boot to configure the hub.
  Serial.begin(115200);                                             // Open USB serial for logs.

  pinMode(GREEN_LED, OUTPUT);                                       // Prepare the green debug LED pin.
  digitalWrite(GREEN_LED, LOW);                                     // Ensure the green LED starts off.
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);                          // Enable pull-up on the reset button pin.

  WiFi.mode(WIFI_STA);                                              // Put the ESP32 into station mode for ESP-NOW.
  if (esp_now_init() != ESP_OK) {                                   // Initialize ESP-NOW and confirm success.
    Serial.println("[Hub] ESP-NOW init failed");                    // Log an error if initialization fails.
    while (true) { delay(1000); }                                   // Halt here forever because networking is required.
  }

  esp_now_register_send_cb(onDataSent);                             // Receive notifications about ESP-NOW sends.
  esp_now_register_recv_cb(onDataRecv);                             // Receive notifications about ESP-NOW packets.

  clearPeers();                                                     // Reset the peer table to empty.
  openRegistrationWindow();                                         // Begin listening for registrations immediately.

  Serial.println("[Hub] Ready. Waiting for BTN_REGISTERs...");     // Announce readiness on the serial console.
}  // End of setup.

// ===================================================
//                         Loop
// ===================================================
void loop() {                                                       // Main loop runs repeatedly after setup.
  // Close registration window and start the round
  if (reg_open && (long)(millis() - reg_deadline_ms) >= 0) {        // If the registration timer expired...
    closeRegistrationAndStartRound();                              // ...transition into the game round.
  }

  if (resetPressedEdge()) {                                        // Check for a manual reset press.
    handleResetButtonPress();                                      // Respond when the button is pressed.
  }

  // Nothing else to poll; all actions are event-driven
}  // End of loop.
