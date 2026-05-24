// ESP8266 - AquaFlow WebSocket Client
// *** VERSION v9.5 ***
// Hardware Serial (pin RX/TX ESP8266)
// Connect ke Railway WebSocket server
//
// FIX v9.2 (ESP):
//  Bug#28. parseFromUNO() tambah parsing tune= field
//  Bug#29. sendDataWS() forward tune=%d ke dashboard
//  Bug#30. loop() WiFi reconnect check setiap 5s
//  Bug#31. rxBuf timeout reset >500ms idle
//
// FIX v2:
//  1. ledFlash() non-blocking (pakai timer millis, bukan delay)
//  2. yield() di loop() supaya background WiFi/TCP tetap jalan
//  3. ESP.wdtFeed() saat connecting WiFi supaya nggak WDT reset
//  4. Serial sync saat ESP baru dinyalakan di tengah UNO running

#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>  // Library: "WebSockets" by Markus Sattler

#define WIFI_SSID    "YOUR_WIFI_SSID"
#define WIFI_PASS    "YOUR_WIFI_PASSWORD"
#define WS_HOST      "aquaflow-ws-production.up.railway.app"
#define WS_PORT      443
#define WS_PATH      "/esp"

#define LED_PIN       2
#define SEND_INTERVAL 500UL

WebSocketsClient wsClient;

// ── Serial receive buffer ──────────────────────────────────────────────────
#define RXBUF_SIZE 200
char    rxBuf[RXBUF_SIZE];
uint8_t rxIdx   = 0;
bool    rxSynced = false;   // FIX 4: true setelah baris valid pertama ditemukan

// ── Shared data ────────────────────────────────────────────────────────────
float  pv = 0, sp = 0;
int    out = 0, running = 0, alHi = 0, alLo = 0;
float  kp = 0, ki = 0, kd = 0;
char   mode = 'P';
int    tunePhase = 0;  // Fix #28/#29: 0=off 1=fill 2=drain 3=relay 4=done 5=cancel
bool   newData = false;
uint32_t lastSend = 0;
uint32_t lastHB   = 0;      // FIX#ESP2: heartbeat timer

// ── FIX 1: LED non-blocking ────────────────────────────────────────────────
uint32_t ledOffAt = 0;

void ledOn()  { digitalWrite(LED_PIN, LOW);  }
void ledOff() { digitalWrite(LED_PIN, HIGH); }

// Langsung nyalain LED, catat kapan harus mati — TIDAK pakai delay
void ledFlash() {
  ledOn();
  ledOffAt = millis() + 50;
}

// Dipanggil setiap loop() — matiin LED kalau sudah waktunya
void ledUpdate() {
  if (ledOffAt && millis() >= ledOffAt) {
    ledOff();
    ledOffAt = 0;
  }
}

// ── Parser dari UNO ───────────────────────────────────────────────────────
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
  tunePhase = gI("tune=");  // Fix #28
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

// FIX 4: cek apakah baris ini valid (dimulai DATA: atau PIDK:)
bool isValidLine(const char* line) {
  return (strncmp(line, "DATA:", 5) == 0 || strncmp(line, "PIDK:", 5) == 0);
}

// ── Kirim ke WebSocket ────────────────────────────────────────────────────
void sendDataWS() {
  // FIX#ESP2: gabung DATA+PIDK jadi 1 frame cegah race condition di browser/observer
  char buf[220];
  snprintf(buf, sizeof(buf),
    "DATA:pv=%.1f,sp=%.1f,out=%d,mode=%c,run=%d,ah=%d,al=%d,tune=%d,kp=%d,ki=%d,kd=%d",
    pv, sp, out, mode, running, alHi, alLo, tunePhase,
    (int)(kp * 100), (int)(ki * 1000), (int)(kd * 1000)
  );
  wsClient.sendTXT(buf);
  ledFlash();
}

void sendHeartbeat() {
  // FIX#ESP2: heartbeat saat tidak ada data baru biar observer tidak timeout
  char hb[32];
  snprintf(hb, sizeof(hb), "HB:pv=%.1f", pv);
  wsClient.sendTXT(hb);
}


void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      ledFlash();  // FIX 1: non-blocking, aman di event handler
      break;
    case WStype_DISCONNECTED:
      break;
    case WStype_TEXT: {
      char* msg = (char*)payload;
      if (strncmp(msg, "CMD:", 4) == 0) {
        Serial.println(msg);
        ledFlash();  // FIX 1: non-blocking
      }
      break;
    }
    default: break;
  }
}

// ── Setup ─────────────────────────────────────────────────────────────────
void setup() {
  pinMode(LED_PIN, OUTPUT);
  ledOff();

  Serial.begin(115200); // Fix#52: match UNO baud rate

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
    delay(300);
    ESP.wdtFeed();  // FIX 3: kasih makan WDT selama tunggu WiFi connect
  }

  if (WiFi.status() == WL_CONNECTED) {
    ledFlash(); ledFlash();
    wsClient.beginSSL(WS_HOST, WS_PORT, WS_PATH);
    wsClient.onEvent(webSocketEvent);
    wsClient.setReconnectInterval(3000);
  }

  lastSend = millis() - SEND_INTERVAL;
}

// ── Loop ──────────────────────────────────────────────────────────────────
void loop() {
  wsClient.loop();

  yield();  // FIX 2: kasih ESP kesempatan urus background WiFi/TCP task

  ledUpdate();  // FIX 1: proses timer LED non-blocking

  uint32_t now = millis();

  // Fix #30: WiFi reconnect check setiap 5s
  static uint32_t wifiCheckMs = 0;
  if (millis() - wifiCheckMs >= 5000) {
    wifiCheckMs = millis();
    if (WiFi.status() != WL_CONNECTED) {
      // Fix#56: tambah delay + wdtFeed setelah disconnect, cegah flood reconnect + WDT reset
      WiFi.disconnect();
      delay(200);          // tunggu disconnect selesai sebelum reconnect
      ESP.wdtFeed();       // cegah WDT reset kalau WiFi.begin() lambat
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
  }

  // Baca dari Arduino UNO via Hardware Serial
  // Fix #31: timeout reset rxBuf >500ms idle (partial msg safety)
  static uint32_t espRxLastMs = 0;
  if (rxIdx > 0 && (millis() - espRxLastMs) > 500UL) rxIdx = 0;

  while (Serial.available()) {
    espRxLastMs = millis();
    char c = Serial.read();
    if (c == '\n' || rxIdx >= RXBUF_SIZE - 1) {
      rxBuf[rxIdx] = 0;
      rxIdx = 0;

      if (rxBuf[0]) {
        // FIX 4: serial sync — buang baris sampai dapat yang valid
        if (!rxSynced) {
          if (isValidLine(rxBuf)) {
            rxSynced = true;  // dari sini semua baris langsung diproses
          }
          // kalau belum valid, buang baris ini dan lanjut tunggu
        }

        if (rxSynced) {
          parseFromUNO(rxBuf);
          parsePIDK(rxBuf);
        }
      }
    } else if (c != '\r') {
      rxBuf[rxIdx++] = c;
    }
  }

  // Kirim ke Railway WebSocket server setiap 500ms
  if (newData && now - lastSend >= SEND_INTERVAL) {
    if (wsClient.isConnected()) {
      lastSend  = now;
      lastHB    = now;   // FIX#ESP2: reset heartbeat timer saat data terkirim
      newData   = false;
      sendDataWS();
    }
  }

  // FIX#ESP2: kirim heartbeat tiap 1 detik kalau tidak ada data baru
  // Cegah observer timeout (espOnline jadi false padahal ESP hidup)
  if (!newData && (now - lastHB >= 1000)) {
    if (wsClient.isConnected()) {
      lastHB = now;
      sendHeartbeat();
    }
  }
}

