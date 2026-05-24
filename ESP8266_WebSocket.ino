// ESP8266 - AquaFlow WebSocket Client
// Ganti SoftwareSerial → Hardware Serial (pin RX/TX ESP)
// Connect ke Railway WebSocket server
//
// FIX#ESP1 — forward tune= field ke WS: observer bisa lihat fase autotune (biasEst, fill, drain, relay)

#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>  // Library: "WebSockets" by Markus Sattler

#define WIFI_SSID    "hai"
#define WIFI_PASS    "Rahasiaku12"
// Ganti dengan URL Railway lo setelah deploy
// Format: "your-app.railway.app"
#define WS_HOST      "your-app.railway.app"
#define WS_PORT      443       // HTTPS/WSS Railway
#define WS_PATH      "/esp"

#define LED_PIN      2
#define SEND_INTERVAL 500UL

WebSocketsClient wsClient;

#define RXBUF_SIZE 200
char rxBuf[RXBUF_SIZE];
uint8_t rxIdx = 0;

float pv=0, sp=0;
int   out=0, running=0, alHi=0, alLo=0, tune=0;  // FIX#ESP1: tambah tune
float kp=0, ki=0, kd=0;
char  mode='P';
bool  newData = false;
uint32_t lastSend = 0;

void ledOn()  { digitalWrite(LED_PIN, LOW); }
void ledOff() { digitalWrite(LED_PIN, HIGH); }
void ledFlash() { ledOn(); delay(50); ledOff(); }

void parseFromUNO(const char* line) {
  if (strncmp(line, "DATA:", 5) != 0) return;
  const char* p = line + 5;
  auto gF = [&](const char* k) -> float {
    const char* f = strstr(p, k); return f ? atof(f + strlen(k)) : 0;
  };
  auto gI = [&](const char* k) -> int {
    const char* f = strstr(p, k); return f ? atoi(f + strlen(k)) : 0;
  };
  auto gC = [&](const char* k) -> char {
    const char* f = strstr(p, k); return f ? *(f + strlen(k)) : '?';
  };
  pv      = gF("pv=");
  sp      = gF("sp=");
  out     = gI("out=");
  running = gI("run=");
  alHi    = gI("ah=");
  alLo    = gI("al=");
  mode    = gC("mode=");
  tune    = gI("tune=");  // FIX#ESP1: forward tune phase ke WS (0=idle,1=fill,2=drain,3=relay,4=done,5=cancel,6=biasEst)
  newData = true;
}

void parsePIDK(const char* line) {
  if (strncmp(line, "PIDK:", 5) != 0) return;
  const char* p = line + 5;
  auto gF = [&](const char* k) -> float {
    const char* f = strstr(p, k); return f ? atof(f + strlen(k)) : 0;
  };
  kp = gF("kp=") / 100.0f;
  ki = gF("ki=") / 1000.0f;
  kd = gF("kd=") / 1000.0f;
}

void sendDataWS() {
  char buf[180];
  snprintf(buf, sizeof(buf),
    "DATA:pv=%.1f,sp=%.1f,out=%d,mode=%c,run=%d,ah=%d,al=%d,tune=%d",  // FIX#ESP1: tambah tune
    pv, sp, out, mode, running, alHi, alLo, tune
  );
  wsClient.sendTXT(buf);

  char buf2[80];
  snprintf(buf2, sizeof(buf2),
    "PIDK:kp=%d,ki=%d,kd=%d,en=11",
    (int)(kp * 100), (int)(ki * 1000), (int)(kd * 1000)
  );
  wsClient.sendTXT(buf2);
  ledFlash();
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      ledFlash(); ledFlash();
      break;
    case WStype_DISCONNECTED:
      break;
    case WStype_TEXT: {
      // Terima CMD dari server, teruskan ke Arduino via Hardware Serial
      char* msg = (char*)payload;
      if (strncmp(msg, "CMD:", 4) == 0) {
        Serial.println(msg);  // kirim ke Arduino
        ledFlash();
      }
      break;
    }
    default: break;
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  ledOff();

  Serial.begin(9600);  // Hardware Serial ke Arduino

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) delay(300);

  if (WiFi.status() == WL_CONNECTED) {
    ledFlash(); ledFlash();
    // WSS (SSL) untuk Railway HTTPS
    wsClient.beginSSL(WS_HOST, WS_PORT, WS_PATH);
    wsClient.onEvent(webSocketEvent);
    wsClient.setReconnectInterval(3000);
  }

  lastSend = millis() - SEND_INTERVAL;
}

void loop() {
  wsClient.loop();
  uint32_t now = millis();

  // Baca dari Arduino
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || rxIdx >= RXBUF_SIZE - 1) {
      rxBuf[rxIdx] = 0; rxIdx = 0;
      if (rxBuf[0]) {
        parseFromUNO(rxBuf);
        parsePIDK(rxBuf);
      }
    } else if (c != '\r') {
      rxBuf[rxIdx++] = c;
    }
  }

  // Kirim data ke WebSocket server
  if (newData && now - lastSend >= SEND_INTERVAL) {
    if (wsClient.isConnected()) {
      lastSend = now;
      newData = false;
      sendDataWS();
    }
  }
}
