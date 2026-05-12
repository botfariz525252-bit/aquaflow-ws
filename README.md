# AquaFlow WebSocket Server

Real-time PID Water Level Controller Dashboard via WebSocket.

## Deploy ke Railway

1. Push folder ini ke GitHub repo baru
2. Buka railway.app → New Project → Deploy from GitHub
3. Pilih repo ini, Railway otomatis detect Node.js
4. Setelah deploy, copy URL Railway (misal: `your-app.railway.app`)
5. Update `WS_HOST` di `ESP8266_WebSocket.ino` dengan URL Railway lo

## Koneksi

- **Browser** → `wss://your-app.railway.app` (WebSocket biasa)
- **ESP8266** → `wss://your-app.railway.app/esp` (endpoint khusus ESP)

## Alur Data

```
Arduino UNO ──(HW Serial 0/1)──> ESP8266 ──(WSS)──> Railway Server ──(WS)──> Browser Dashboard
     ^                                                       |
     └──────────────────── CMD: ────────────────────────────┘
```

## Library ESP yang dibutuhkan
- `WebSockets` by Markus Sattler (install via Arduino Library Manager)
- `ESP8266WiFi` (built-in ESP8266 board package)
