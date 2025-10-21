// ================== BUTTON_V1.3 ==================
// Pins:
//   BUTTON_PIN = 23 (INPUT_PULLUP, active LOW)
//   LED_PIN    = 12 (indicator LED)
//   DIP        = {4,16,17,5,18,19} -> value 0..63; all ON (63) = FFA
//
// Behavior:
//   - Hardcoded Hub MAC
//   - On boot: add hub as peer
//   - On HUB_LED_CONTROL(on=true, suspend_rx=true): LED ON and suspend hub listening
//     until *this* button is pressed; press clears suspend and sends BTN_PRESS
//   - Obeys FEEDBACK / ROUND_RESET / ALL_FLASH
//   - In FFA: LED idles ON, first CORRECT locks this button for the round
// ================================================================

#include <esp_now.h>
#include <WiFi.h>
#if __has_include(<esp_wifi.h>)
  #include <esp_wifi.h>
  #define HAVE_ESP_WIFI 1
#endif

// ---------- Radio ----------
#define WIFI_CHANNEL 1  // Must match the hub

// ---------- Hardcoded Hub MAC ----------
static const uint8_t HUB_MAC[6] = {0xCC,0xDB,0xA7,0x2D,0xD2,0x48};

// ---------- IO ----------
#define BUTTON_PIN 23
#define LED_PIN    12
constexpr uint8_t DIP_PINS[] = {4,16,17,5,18,19};
constexpr uint8_t DIP_COUNT = sizeof(DIP_PINS)/sizeof(DIP_PINS[0]);

// ---------- Polarity ----------
#define BUTTON_ACTIVE_LOW true
#define LED_ACTIVE_LOW    false
constexpr bool DIP_ACTIVE_LOW = true;
inline void ledOn(){  digitalWrite(LED_PIN, LED_ACTIVE_LOW? LOW:HIGH); }
inline void ledOff(){ digitalWrite(LED_PIN, LED_ACTIVE_LOW? HIGH:LOW); }

// ---------- Debounce ----------
const unsigned long DEBOUNCE_MS = 30;
int lastReading, debouncedState;
unsigned long lastDebounceTime=0;
const int PRESSED_LEVEL = BUTTON_ACTIVE_LOW? LOW:HIGH;
const int IDLE_LEVEL    = BUTTON_ACTIVE_LOW? HIGH:LOW;

// ---------- Wire Protocol ----------
enum MsgKind : uint8_t {
  BTN_REGISTER=1, BTN_PRESS=2,
  HUB_LED_CONTROL=100, HUB_FEEDBACK=101, HUB_ROUND_RESET=102, HUB_ALL_FLASH=103
};
enum FeedbackCode : uint8_t { FDBK_INCORRECT=0, FDBK_CORRECT=1, FDBK_ALREADYCOUNT=2, FDBK_LOCKEDOUT=3 };
typedef struct __attribute__((packed)) { uint8_t kind; uint8_t button_id; bool ffa; uint8_t mac[6]; } btn_register_t;
typedef struct __attribute__((packed)) { uint8_t kind; uint8_t button_id; bool pressed; uint16_t press_id; uint8_t mac[6]; } btn_press_t;
typedef struct __attribute__((packed)) { uint8_t kind; bool on; bool suspend_rx; } hub_led_control_t;
typedef struct __attribute__((packed)) { uint8_t kind; uint8_t result; } hub_feedback_t;
typedef struct __attribute__((packed)) { uint8_t kind; } hub_round_reset_t;
typedef struct __attribute__((packed)) { uint8_t kind; uint8_t times; uint16_t on_ms; uint16_t off_ms; } hub_all_flash_t;

// ---------- State ----------
uint8_t myMac[6]={0};
uint8_t button_id=0;
bool ffa=false;
bool locked=false;
bool listeningSuspended=false;
uint16_t press_id=0;
unsigned long lastPressTxMs=0;
const unsigned long PRESS_COOLDOWN_MS=150;

// ---------- Helpers ----------
uint8_t readDip(){
  uint8_t v=0;
  for(uint8_t i=0;i<DIP_COUNT;i++){
    int s=digitalRead(DIP_PINS[i]);
    if (DIP_ACTIVE_LOW) s=!s;
    if (s) v|=(1u<<i);
  }
  return v;
}
bool pressedEdge(){
  int reading=digitalRead(BUTTON_PIN);
  if (reading!=lastReading){ lastDebounceTime=millis(); lastReading=reading; }
  if ((millis()-lastDebounceTime)>DEBOUNCE_MS){
    if (reading!=debouncedState){
      debouncedState=reading;
      if (debouncedState==PRESSED_LEVEL) return true;
    }
  }
  return false;
}
String macStr(const uint8_t m[6]){ char b[18]; snprintf(b,sizeof(b),"%02X:%02X:%02X:%02X:%02X:%02X",m[0],m[1],m[2],m[3],m[4],m[5]); return String(b); }

// ---------- ESP-NOW ----------
static inline esp_err_t sendNow(const uint8_t* mac, const void* buf, size_t len){ return esp_now_send(mac,(const uint8_t*)buf,len); }
void addHubPeer(){
  esp_now_peer_info_t p{}; memcpy(p.peer_addr,HUB_MAC,6); p.channel=WIFI_CHANNEL; p.encrypt=false;
  esp_now_add_peer(&p);
}

// ---------- Packet handling ----------
void handleHubPacket(const uint8_t* data, int len){
  if (len<1) return;
  uint8_t kind=data[0];
  if (listeningSuspended && kind!=HUB_ROUND_RESET) return;

  switch(kind){
    case HUB_LED_CONTROL: {
      if (len<(int)sizeof(hub_led_control_t)) return;
      auto* m=(const hub_led_control_t*)data;
      if (m->on) ledOn(); else ledOff();
      listeningSuspended = m->suspend_rx;
      locked = false;
      break;
    }
    case HUB_FEEDBACK: {
      if (len<(int)sizeof(hub_feedback_t)) return;
      auto* m=(const hub_feedback_t*)data;
      if (m->result==FDBK_CORRECT){ if (ffa){ ledOff(); locked=true; } Serial.println("[Button] CORRECT"); }
      else if (m->result==FDBK_INCORRECT){ Serial.println("[Button] INCORRECT"); }
      else if (m->result==FDBK_ALREADYCOUNT){ if (ffa){ ledOff(); locked=true; } Serial.println("[Button] ALREADY"); }
      else if (m->result==FDBK_LOCKEDOUT){ locked=true; ledOff(); Serial.println("[Button] LOCKED OUT"); }
      break;
    }
    case HUB_ROUND_RESET: {
      if (len<(int)sizeof(hub_round_reset_t)) return;
      locked=false; listeningSuspended=false; Serial.println("[Button] Reset received");
      if (ffa) ledOn(); else ledOff();
      break;
    }
    case HUB_ALL_FLASH: {
      if (len<(int)sizeof(hub_all_flash_t)) return;
      auto* m=(const hub_all_flash_t*)data;
      for (uint8_t i=0;i<m->times;i++){ ledOn(); delay(m->on_ms); ledOff(); delay(m->off_ms); }
      locked=true; listeningSuspended=false;
      break;
    }
    default: break;
  }
}

// Compatibility wrapper for different IDF versions
static void handleRecvCommon(const uint8_t* src, const uint8_t* data, int len){
  if (memcmp(src,HUB_MAC,6)!=0) return; // only listen to our hub
  handleHubPacket(data, len);
}

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
void onDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status){ (void)info; (void)status; }
void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len){ handleRecvCommon(info->src_addr, data, len); }
#else
void onDataSent(const uint8_t* mac, esp_now_send_status_t status){ (void)mac; (void)status; }
void onDataRecv(const uint8_t* mac, const uint8_t* data, int len){ handleRecvCommon(mac, data, len); }
#endif

// ---------- Setup ----------
void setup(){
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT); ledOff();
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  for (uint8_t p: DIP_PINS) pinMode(p, INPUT_PULLUP);

  lastReading = debouncedState = IDLE_LEVEL;

  button_id = readDip();
  ffa = (button_id == ((1u<<DIP_COUNT)-1));
  Serial.printf("Button ID=%u  FFA=%s  Hub=%s\n", button_id, ffa?"YES":"NO", macStr(HUB_MAC).c_str());
  if (ffa) ledOn();

#if defined(HAVE_ESP_WIFI)
  esp_wifi_get_mac(WIFI_IF_STA, myMac);
#else
  WiFi.macAddress(myMac);
#endif

  WiFi.mode(WIFI_STA);
#if defined(HAVE_ESP_WIFI)
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
#endif

  if (esp_now_init()!=ESP_OK){ Serial.println("ESP-NOW init failed"); while(true) delay(1000); }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  addHubPeer();

  // Send REGISTER once at boot
  btn_register_t r{}; r.kind=BTN_REGISTER; r.button_id=button_id; r.ffa=ffa; memcpy(r.mac,myMac,6);
  if (sendNow(HUB_MAC,&r,sizeof(r))==ESP_OK) Serial.println("[Button] REGISTER sent");
  else Serial.println("[Button] REGISTER failed");
}

// ---------- Loop ----------
bool cooldownOK(){
  unsigned long now=millis();
  if (lastPressTxMs && (now-lastPressTxMs)<PRESS_COOLDOWN_MS) return false;
  lastPressTxMs = now; return true;
}

void loop(){
  if (pressedEdge()){
    if (locked){ Serial.println("[Button] Press ignored (locked)"); return; }
    if (!cooldownOK()) return;

    if (!ffa) ledOff();           // in SEQUENCE, darken immediately
    listeningSuspended=false;     // re-open hub listening

    btn_press_t m{};
    m.kind=BTN_PRESS; m.button_id=button_id; m.pressed=true; m.press_id=++press_id; memcpy(m.mac,myMac,6);
    esp_err_t st = sendNow(HUB_MAC, &m, sizeof(m));
    Serial.printf("[Button] PRESS id=%u %s\n", m.press_id, (st==ESP_OK)?"sent":"fail");
  }
}
