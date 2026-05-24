// AquaFlow WS Server v10.5.3 — FIX#S3: WebSocket keepalive ping/pong
const express = require('express');
const { WebSocketServer, WebSocket } = require('ws');
const http = require('http');
const path = require('path');

const app = express();
const server = http.createServer(app);
const wss = new WebSocketServer({ server });

app.use(express.static(path.join(__dirname, 'public')));

// State data dari Arduino/ESP
let state = {
  pv: 0, sp: 0, out: 0, mode: 'P',
  running: 0, alarmHi: 0, alarmLo: 0,
  kp: 0, ki: 0, kd: 0,
  tune: 0,
  online: false,
  lastSeen: null
};

let cmdQueue = [];
let cmdIdCounter = 0;
let cmdLog = [];

// Broadcast ke semua browser client
function broadcast(msg) {
  const data = JSON.stringify(msg);
  wss.clients.forEach(client => {
    if (client.readyState === WebSocket.OPEN && client._type === 'browser') {
      client.send(data);
    }
  });
}

// Kirim command ke ESP
function sendToESP(cmdStr) {
  wss.clients.forEach(client => {
    if (client.readyState === WebSocket.OPEN && client._type === 'esp') {
      client.send('CMD:' + cmdStr);
    }
  });
}

// Offline check — tandai offline kalau 10 detik tidak ada data
setInterval(() => {
  if (state.online && state.lastSeen && Date.now() - state.lastSeen > 10000) {  // FIX#S2: timeout 5s→10s
    state.online = false;
    broadcast({ type: 'state', data: state });
  }
}, 1000);

// FIX#S3: WebSocket keepalive — Railway/cloud proxy drop idle WS setelah ~12s
// Kirim ping frame ke semua client (ESP + browser) setiap 8 detik
setInterval(() => {
  wss.clients.forEach(client => {
    if (client.readyState === WebSocket.OPEN) {
      client.ping();
    }
  });
}, 8000);

wss.on('connection', (ws, req) => {
  const url = req.url || '';

  // ESP connect ke /esp, browser connect ke /
  if (url.includes('/esp')) {
    ws._type = 'esp';
    console.log('[ESP] Connected');

    ws.on('message', (raw) => {
      const line = raw.toString().trim();
      console.log('[ESP]', line);

      if (line.startsWith('DATA:')) {
        const p = line.slice(5);
        const gF = (k) => { const m = p.match(new RegExp(k + '=([\\d.\\-]+)')); return m ? parseFloat(m[1]) : 0; };
        const gI = (k) => { const m = p.match(new RegExp(k + '=([\\d\\-]+)')); return m ? parseInt(m[1]) : 0; };
        const gC = (k) => { const m = p.match(new RegExp(k + '=([A-Za-z])')); return m ? m[1] : '?'; };

        state.pv      = gF('pv');
        state.sp      = gF('sp');
        state.out     = gI('out');
        state.mode    = gC('mode');
        state.running = gI('run');
        state.alarmHi = gI('ah');
        state.alarmLo = gI('al');
        state.tune    = gI('tune');
        state.online  = true;
        state.lastSeen = Date.now();

        broadcast({ type: 'state', data: state });
      }

      if (line.startsWith('PIDK:')) {
        const p = line.slice(5);
        const gF = (k) => { const m = p.match(new RegExp(k + '=([\\d.\\-]+)')); return m ? parseFloat(m[1]) : 0; };
        state.kp = gF('kp') / 100;
        state.ki = gF('ki') / 1000;
        state.kd = gF('kd') / 1000;
        state.lastSeen = Date.now();  // FIX#S1: update lastSeen saat PIDK diterima
        broadcast({ type: 'state', data: state });
      }
    });

    ws.on('close', () => {
      console.log('[ESP] Disconnected');
      state.online = false;
      broadcast({ type: 'state', data: state });
    });

  } else {
    ws._type = 'browser';
    console.log('[Browser] Connected');

    // Kirim state awal dan log
    ws.send(JSON.stringify({ type: 'state', data: state }));
    ws.send(JSON.stringify({ type: 'cmdlog', data: cmdLog.slice(-20) }));

    ws.on('message', (raw) => {
      let msg;
      try { msg = JSON.parse(raw.toString()); } catch(e) { return; }

      if (msg.type === 'cmd') {
        const { cmd } = msg;
        const id = ++cmdIdCounter;
        const ts = new Date().toLocaleTimeString('id-ID');
        const logEntry = { id, cmd, ts, status: 'SENT' };
        cmdLog.push(logEntry);
        if (cmdLog.length > 50) cmdLog.shift();

        sendToESP(cmd);
        broadcast({ type: 'cmdlog', data: cmdLog.slice(-20) });
        console.log('[CMD]', cmd);
      }
    });

    ws.on('close', () => console.log('[Browser] Disconnected'));
  }
});

const PORT = process.env.PORT || 3000;
server.listen(PORT, () => {
  console.log(`AquaFlow WS Server running on port ${PORT}`);
});
