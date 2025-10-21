// ================== HUB_V1.4 ==================
// Transport: ESP-NOW (station mode, fixed channel)
// Board: ESP32-WROOM-32
//
// GPIOs:
//   - RELAY_PIN  = GPIO12 (active HIGH)
//   - RESET_PIN  = GPIO18 (INPUT_PULLUP, active LOW)
//
// Behavior:
//   - Registration window on boot
//   - SEQUENCE mode: hub lights exactly one button at a time (ascending button_id).
//     LED_ON is sent with suspend_rx=true; the button ignores hub except RESET until it’s pressed.
//   - FFA mode: hub turns all LEDs on; each counts once; finish when all have pressed
//   - Lockout between rounds
//   - Relay energizes for 1s at end-of-round
//
// Important change:
//   - At round start, we now broadcast HUB_ROUND_RESET *before* arming the first LED,
//     fixing the “button #1 LED flicker”.
// ================================================================

#include <esp_now.h>
#include <WiFi.h>
#if __has_include(<esp_wifi.h>)
  #include <esp_wifi.h>
  #define HAVE_ESP_WIFI 1
#endif

// ---------- Radio ----------
#define WIFI_CHANNEL 1  // Must match the BUTTON sketch

// ---------- Pins ----------
#define RELAY_PIN        12
#define RESET_BUTTON_PIN 18

// ---------- Reset handling ----------
enum ResetEvent : uint8_t { RESET_EVENT_NONE=0, RESET_EVENT_TAP=1, RESET_EVENT_HOLD=2 };
const bool RESET_ACTIVE_LOW = true;
const unsigned long RESET_DEBOUNCE_MS = 40;
const unsigned long RESET_HOLD_THRESHOLD_MS = 1000;
const int RESET_PRESSED_LEVEL  = RESET_ACTIVE_LOW ? LOW  : HIGH;
const int RESET_RELEASED_LEVEL = RESET_ACTIVE_LOW ? HIGH : LOW;

int resetLastReading = RESET_RELEASED_LEVEL;
int resetStableState = RESET_RELEASED_LEVEL;
unsigned long resetLastDebounceMs = 0;
bool resetPressActive=false, resetHoldFired=false, resetHoldActive=false;
unsigned long resetPressStartMs = 0;

// ===================================================
//                Wire Protocol (ESP-NOW)
// ===================================================
enum MsgKind : uint8_t {
  BTN_REGISTER    = 1,
  BTN_PRESS       = 2,
  HUB_LED_CONTROL = 100,
  HUB_FEEDBACK    = 101,
  HUB_ROUND_RESET = 102,
  HUB_ALL_FLASH   = 103,
};

enum FeedbackCode : uint8_t {
  FDBK_INCORRECT=0, FDBK_CORRECT=1, FDBK_ALREADYCOUNT=2, FDBK_LOCKEDOUT=3,
};

typedef struct __attribute__((packed)) { uint8_t kind; uint8_t button_id; bool ffa; uint8_t mac[6]; } btn_register_t;
typedef struct __attribute__((packed)) { uint8_t kind; uint8_t button_id; bool pressed; uint16_t press_id; uint8_t mac[6]; } btn_press_t;
typedef struct __attribute__((packed)) { uint8_t kind; bool on; bool suspend_rx; } hub_led_control_t;
typedef struct __attribute__((packed)) { uint8_t kind; uint8_t result; } hub_feedback_t;
typedef struct __attribute__((packed)) { uint8_t kind; } hub_round_reset_t;
typedef struct __attribute__((packed)) { uint8_t kind; uint8_t times; uint16_t on_ms; uint16_t off_ms; } hub_all_flash_t;

// ===================================================
//                Registration / Peers
// ===================================================
struct Peer {
  bool present=false;
  uint8_t mac[6]={0};
  uint8_t button_id=0;
  bool ffa=false;
  bool counted=false;
  uint16_t last_press_id=0;
};
static const int MAX_PEERS=16;
static const uint8_t MAX_BUTTON_ID=63;
Peer peers[MAX_PEERS];
int idToIndex[MAX_BUTTON_ID+1];

int findPeerByMac(const uint8_t mac[6]) {
  for (int i=0;i<MAX_PEERS;i++) if (peers[i].present && memcmp(peers[i].mac,mac,6)==0) return i;
  return -1;
}
int ensurePeerSlot(const uint8_t mac[6]) {
  int idx=findPeerByMac(mac);
  if (idx>=0) return idx;
  for (int i=0;i<MAX_PEERS;i++) if (!peers[i].present) {
    peers[i]=Peer{};
    peers[i].present=true;
    memcpy(peers[i].mac,mac,6);
    return i;
  }
  return -1;
}
void clearPeers(){ for(int i=0;i<MAX_PEERS;i++) peers[i]=Peer{}; for(int i=0;i<=MAX_BUTTON_ID;i++) idToIndex[i]=-1; }
void rebuildIdMap(){ for(int i=0;i<=MAX_BUTTON_ID;i++) idToIndex[i]=-1; for(int i=0;i<MAX_PEERS;i++) if(peers[i].present){ uint8_t id=peers[i].button_id; if(id>0 && id<=MAX_BUTTON_ID) idToIndex[id]=i; } }
int countPresent(){ int n=0; for(int i=0;i<MAX_PEERS;i++) if(peers[i].present) n++; return n; }
bool allFFA(){ bool any=false; for(int i=0;i<MAX_PEERS;i++){ if(!peers[i].present) continue; any=true; if(!peers[i].ffa) return false; } return any; }
void resetRoundRuntime(){ for(int i=0;i<MAX_PEERS;i++) if(peers[i].present) peers[i].counted=false; }

// ===================================================
//                 ESP-NOW helpers
// ===================================================
static esp_err_t sendTo(const uint8_t mac[6], const void* buf, size_t len){ return esp_now_send(mac,(const uint8_t*)buf,len); }
static void addPeerIfNeeded(const uint8_t mac[6]) {
  esp_now_peer_info_t p{}; memcpy(p.peer_addr,mac,6); p.channel=WIFI_CHANNEL; p.encrypt=false;
  esp_now_add_peer(&p);
}
static void sendFeedback(const uint8_t mac[6], uint8_t code){
  hub_feedback_t m{}; m.kind=HUB_FEEDBACK; m.result=code; addPeerIfNeeded(mac); sendTo(mac,&m,sizeof(m));
}
static void broadcastRoundReset(){
  hub_round_reset_t m{}; m.kind=HUB_ROUND_RESET;
  for(int i=0;i<MAX_PEERS;i++) if(peers[i].present){ addPeerIfNeeded(peers[i].mac); sendTo(peers[i].mac,&m,sizeof(m)); }
}
static void sendLedControl(const uint8_t mac[6], bool on, bool suspend){
  hub_led_control_t m{}; m.kind=HUB_LED_CONTROL; m.on=on; m.suspend_rx=suspend;
  addPeerIfNeeded(mac); sendTo(mac,&m,sizeof(m));
}
static void sendAllFlash(uint8_t times=5, uint16_t on_ms=200, uint16_t off_ms=200){
  hub_all_flash_t m{}; m.kind=HUB_ALL_FLASH; m.times=times; m.on_ms=on_ms; m.off_ms=off_ms;
  for(int i=0;i<MAX_PEERS;i++) if(peers[i].present){ addPeerIfNeeded(peers[i].mac); sendTo(peers[i].mac,&m,sizeof(m)); }
}
static void allLedsOff(){ for(int i=0;i<MAX_PEERS;i++) if(peers[i].present) sendLedControl(peers[i].mac,false,false); }

// ===================================================
//                    Game State
// ===================================================
enum GameMode : uint8_t { MODE_IDLE=0, MODE_SEQUENCE=1, MODE_FFA=2 };
GameMode mode=MODE_IDLE;
int participants=0, counted=0;

int sequenceOrder[MAX_PEERS];
int sequenceCount=0;
int activeSequenceIndex=-1;

bool reg_open=false;
unsigned long reg_deadline_ms=0;
const uint32_t REG_WINDOW_MS=3000;

bool locked_out=false;

void rebuildSequenceOrder(){
  rebuildIdMap();
  sequenceCount=0;
  for(uint8_t id=1; id<=MAX_BUTTON_ID; id++){
    int idx=idToIndex[id];
    if (idx>=0 && peers[idx].present) sequenceOrder[sequenceCount++]=idx;
  }
}
bool armSequenceIndex(int seqIndex){
  if (seqIndex<0 || seqIndex>=sequenceCount) return false;
  int peerIdx = sequenceOrder[seqIndex];
  if (peerIdx<0 || !peers[peerIdx].present) return false;
  sendLedControl(peers[peerIdx].mac, true, true); // LED ON + suspend_rx
  activeSequenceIndex=seqIndex;
  Serial.printf("[Hub] Lighting button id=%u (seq %d/%d)\n", peers[peerIdx].button_id, seqIndex+1, sequenceCount);
  return true;
}

// ===================================================
//                  Game Flow
// ===================================================
void openRegistrationWindow(){
  reg_open=true; locked_out=false; reg_deadline_ms=millis()+REG_WINDOW_MS;
  mode=MODE_IDLE; participants=0; counted=0; sequenceCount=0; activeSequenceIndex=-1;
  resetRoundRuntime();
  digitalWrite(RELAY_PIN, HIGH); delay(30); digitalWrite(RELAY_PIN, LOW); // tiny "heartbeat"
  Serial.println("[Hub] Registration window OPEN");
}

bool startRoundFromPeers(){
  reg_open=false;
  participants = countPresent();
  if (participants<=0){ Serial.println("[Hub] No participants; staying idle."); mode=MODE_IDLE; return false; }
  resetRoundRuntime(); counted=0; activeSequenceIndex=-1; sequenceCount=0; locked_out=false;

  if (allFFA()){
    mode=MODE_FFA; rebuildIdMap();
    Serial.printf("[Hub] Starting FFA with %d participants.\n", participants);

    // Send RESET first, then turn LEDs on
    broadcastRoundReset();
    delay(10);
    for (int i=0;i<MAX_PEERS;i++) if (peers[i].present) sendLedControl(peers[i].mac, true, false);

  } else {
    mode=MODE_SEQUENCE; rebuildSequenceOrder();
    Serial.printf("[Hub] Starting SEQUENCE with %d participants.\n", participants);
    if (sequenceCount==0){ Serial.println("[Hub] Warning: no sequence-capable buttons."); mode=MODE_IDLE; return false; }

    // Send RESET first, then clean LEDs, then arm #1
    broadcastRoundReset();
    delay(10);
    allLedsOff();
    if (!armSequenceIndex(0)){ Serial.println("[Hub] Failed to arm first sequence button."); mode=MODE_IDLE; return false; }
  }
  return true;
}

void finishRound(){
  Serial.println("[Hub] Round complete. Awaiting reset...");
  sendAllFlash(5,200,200);                  // ~2s of flashing
  broadcastRoundReset();
  mode=MODE_IDLE; counted=0; reg_open=false; locked_out=true; activeSequenceIndex=-1; sequenceCount=0;

  // Relay ON for 1s per your request
  digitalWrite(RELAY_PIN, HIGH);
  delay(1000);
  digitalWrite(RELAY_PIN, LOW);
}

// ===================================================
// Reset button polling
// ===================================================
ResetEvent pollResetButton(){
  int reading = digitalRead(RESET_BUTTON_PIN);
  if (reading != resetLastReading){ resetLastDebounceMs=millis(); resetLastReading=reading; }
  if ((millis()-resetLastDebounceMs) > RESET_DEBOUNCE_MS){
    if (reading != resetStableState){
      resetStableState=reading;
      if (resetStableState==RESET_PRESSED_LEVEL){
        resetPressActive=true; resetHoldFired=false; resetHoldActive=false; resetPressStartMs=millis();
      } else {
        if (resetPressActive){
          unsigned long held=millis()-resetPressStartMs; bool fired=resetHoldFired;
          resetPressActive=false; resetHoldActive=false; resetHoldFired=false;
          if (!fired && held<RESET_HOLD_THRESHOLD_MS) return RESET_EVENT_TAP;
        }
      }
    }
  }
  if (resetPressActive && !resetHoldFired && (millis()-resetPressStartMs)>=RESET_HOLD_THRESHOLD_MS){
    resetHoldFired=true; resetHoldActive=true; return RESET_EVENT_HOLD;
  }
  return RESET_EVENT_NONE;
}

void handleShortResetPress(){
  Serial.println("[Hub] Reset button tapped.");
  locked_out=false; counted=0; resetRoundRuntime(); activeSequenceIndex=-1; sequenceCount=0;
  // No broadcast here; startRoundFromPeers() now handles RESET->LED_ON ordering
  if (!startRoundFromPeers()) openRegistrationWindow();
}
void handleResetHold(){
  Serial.println("[Hub] Reset button held; reopening registration.");
  locked_out=false; counted=0; resetRoundRuntime(); broadcastRoundReset();
  mode=MODE_IDLE; reg_open=true; reg_deadline_ms=millis()+REG_WINDOW_MS; participants=countPresent();
  rebuildSequenceOrder();
}

// ===================================================
// ESP-NOW callbacks
// ===================================================
static void handleRecvCommon(const uint8_t* src, const uint8_t* data, int len){
  if (len<1) return;
  uint8_t kind = data[0];

  switch(kind){
    case BTN_REGISTER: {
      if (len<(int)sizeof(btn_register_t)) return;
      const btn_register_t* m = (const btn_register_t*)data;
      if (!reg_open && mode==MODE_IDLE) openRegistrationWindow();
      if (!reg_open){ Serial.println("[Hub] Ignoring BTN_REGISTER (round in progress)."); return; }
      int idx=ensurePeerSlot(src); if (idx<0){ Serial.println("[Hub] Peer table full; registration dropped."); return; }
      peers[idx].button_id = m->button_id; peers[idx].ffa = m->ffa; addPeerIfNeeded(src);
      Serial.printf("[Hub] REGISTER %02X:%02X:%02X:%02X:%02X:%02X id=%u ffa=%s\n",
        src[0],src[1],src[2],src[3],src[4],src[5], m->button_id, m->ffa?"YES":"NO");
      reg_deadline_ms = millis()+REG_WINDOW_MS;
      break;
    }
    case BTN_PRESS: {
      if (len<(int)sizeof(btn_press_t)) return;
      const btn_press_t* m = (const btn_press_t*)data;
      if (!m->pressed) return;
      if (locked_out){ Serial.println("[Hub] Press ignored (locked out)."); sendFeedback(src,FDBK_LOCKEDOUT); return; }
      if (mode==MODE_IDLE && !reg_open){ openRegistrationWindow(); Serial.println("[Hub] Press opened registration; ignored."); return; }
      if (reg_open){ Serial.println("[Hub] Press during registration; ignored."); return; }

      int idx=findPeerByMac(src);
      if (idx<0 || !peers[idx].present){ Serial.println("[Hub] Press from unknown MAC; ignored."); return; }
      if (m->press_id!=0 && m->press_id==peers[idx].last_press_id) return; // de-dup
      peers[idx].last_press_id = m->press_id;

      if (mode==MODE_SEQUENCE){
        if (activeSequenceIndex<0 || activeSequenceIndex>=sequenceCount){
          Serial.println("[Hub] Sequence press but no active target.");
          sendFeedback(src, FDBK_INCORRECT);
          return;
        }
        int expectedPeerIdx = sequenceOrder[activeSequenceIndex];
        uint8_t expectedButtonId = peers[expectedPeerIdx].button_id;
        uint8_t id = m->button_id;
        Serial.printf("[Hub] SEQ press id=%u expected=%u\n", id, expectedButtonId);
        if (idx==expectedPeerIdx && id==expectedButtonId){
          sendFeedback(src, FDBK_CORRECT);
          sendLedControl(src, false, false); // turn that LED off
          activeSequenceIndex++;
          if (activeSequenceIndex>=sequenceCount) finishRound();
          else armSequenceIndex(activeSequenceIndex);
        } else {
          sendFeedback(src, FDBK_INCORRECT);
          Serial.println("[Hub] Incorrect press. Locking round until reset.");
          locked_out=true; allLedsOff();
        }
      } else if (mode==MODE_FFA){
        Serial.printf("[Hub] FFA press from %02X:%02X:%02X:%02X:%02X:%02X\n",src[0],src[1],src[2],src[3],src[4],src[5]);
        if (!peers[idx].counted){ peers[idx].counted=true; counted++; sendFeedback(src,FDBK_CORRECT); sendLedControl(src,false,false); if (counted>=participants) finishRound(); }
        else { sendFeedback(src,FDBK_ALREADYCOUNT); sendLedControl(src,false,false); }
      } else {
        sendFeedback(src,FDBK_INCORRECT);
      }
      break;
    }
    default: break;
  }
}

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
void onDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status){ (void)info; (void)status; }
void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len){ handleRecvCommon(info->src_addr, data, len); }
#else
void onDataSent(const uint8_t* mac, esp_now_send_status_t status){ (void)mac; (void)status; }
void onDataRecv(const uint8_t* mac, const uint8_t* data, int len){ handleRecvCommon(mac, data, len); }
#endif

// ===================================================
// Setup / Loop
// ===================================================
void setup(){
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT); digitalWrite(RELAY_PIN, LOW);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
#if defined(HAVE_ESP_WIFI)
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
#endif

  if (esp_now_init()!=ESP_OK){ Serial.println("[Hub] ESP-NOW init failed"); while(true) delay(1000); }
  esp_now_register_send_cb(onDataSent);
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
  esp_now_register_recv_cb(onDataRecv);
#else
  esp_now_register_recv_cb(onDataRecv);
#endif

  clearPeers();
  openRegistrationWindow();
  Serial.println("[Hub] Ready. Waiting for BTN_REGISTERs...");
}

void loop(){
  if (reg_open && (long)(millis()-reg_deadline_ms)>=0) startRoundFromPeers();

  ResetEvent ev = pollResetButton();
  if (ev==RESET_EVENT_TAP) handleShortResetPress();
  else if (ev==RESET_EVENT_HOLD) handleResetHold();

  if (resetHoldActive && reg_open) reg_deadline_ms = millis()+REG_WINDOW_MS;
}
