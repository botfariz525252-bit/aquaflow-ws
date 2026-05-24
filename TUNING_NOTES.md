# AquaFlow — Tuning Notes & Known Issues

## Masalah: Autotune ZN lambat / tidak selesai saat outflow sedang-tinggi

### Gejala
- Autotune berjalan sangat lama (>5 menit) tanpa selesai
- PV naik sangat lambat saat relay HIGH
- Siklus oscillasi tidak simetris (naik pelan, turun cepat)

### Root Cause
ZN relay feedback butuh tangki bisa **oscillate naik-turun simetris**.

Saat autotune, relay toggle antara:
- `HIGH = TUN_BIAS + TUN_AMP` → pompa aktif → tangki naik
- `LOW  = TUN_BIAS - TUN_AMP` → pompa off → tangki turun (outflow saja)

Kalau outflow ≈ input pompa (net flow ≈ 0 saat relay HIGH), tangki hampir tidak naik:
```
Contoh: pompa 5 LPM, outflow 4 LPM
Relay HIGH (100%): net flow = +1 LPM → naik sangat lambat
Relay LOW  (0%):   net flow = -4 LPM → turun cepat
```
Hasilnya: Pu yang diukur tidak akurat → Kp/Ki hasil autotune salah.

### Solusi yang Direncanakan (TODO)

**Auto-Bias Estimation sebelum relay start:**

1. Saat autotune dimulai, jalankan **outflow estimation phase** dulu:
   - Pompa 100% selama 5 detik → ukur rate naik PV (dPV_up)
   - Pompa 0% selama 5 detik → ukur rate turun PV (dPV_down)
   - Estimasi outflow% = `dPV_down / (dPV_up + dPV_down) × 100`

2. Set `TUN_BIAS = outflow% + 20%` (margin supaya relay LOW masih bisa turunkan tangki)

3. Baru mulai relay oscillation dengan bias yang sudah di-adjust

**Formula bias optimal:**
```
outflow_rate = abs(dPV_down) / dt   // %/detik
inflow_rate  = abs(dPV_up)   / dt   // %/detik
bias_optimal = outflow_rate / (inflow_rate + outflow_rate) × 100 + 20
bias_optimal = constrain(bias_optimal, 30, 70)
```

### Workaround Sementara
Naikkan `TUN_AMP` di menu autotune dari default 20% ke 35-40%.
Ini memperbesar relay swing sehingga relay HIGH lebih dominan vs outflow.

---

## Formula PID: ZN Classic vs Tyreus-Luyben

Sistem tangki AquaFlow bersifat **asimetris**:
- Naik: cepat (pompa aktif, 5 LPM)
- Turun: lambat (gravitasi/outflow konstan)

ZN Classic dirancang untuk sistem simetris → terlalu agresif → PV overshoot dan nyangkut di atas SP.

| Formula | Kp (PI) | Ti (PI) | Kp (PID) | Ti (PID) | Td (PID) |
|---------|---------|---------|----------|---------|---------|
| ZN Classic | 0.45×Ku | 0.83×Pu | 0.60×Ku | 0.50×Pu | 0.125×Pu |
| Tyreus-Luyben | 0.316×Ku | 2.2×Pu | 0.455×Ku | 2.2×Pu | Pu/6.3 |
| **TL + de-tune 0.6x (v10.5.4)** | **0.190×Ku** | **2.2×Pu** | **0.273×Ku** | **2.2×Pu** | **Pu/6.3** |

De-tune 0.6x diterapkan di v10.5.4 karena outflow sistem bersifat sedang (40-60% pump rate).

---

## PID Deadband & Integrator (v10.5.5)

### Masalah: PV drift ±2% dari SP — kadang kurang, kadang lebih

#### Root Cause 1: Deadband terlalu lebar (2.5%)
- Deadband 2.5% → PID nganggur selama PV dalam range ±2.5% dari SP
- PV bisa settle di mana saja dalam range itu → bisa -2% atau +2% dari SP
- **Fix#P7 (v10.5.5):** Deadband diperkecil dari 2.5% → **1.0%**

#### Root Cause 2: Integrator freeze → outflow tidak terkompensasi
- Saat PV dalam deadband, integrator di-freeze total (e=0)
- Outflow terus berjalan → tidak ada yang mengkompensasi → PV perlahan drift turun
- Kalau PV overshoot masuk deadband dari atas, decay 0.9999 terlalu lambat → PV nyangkut di atas SP
- **Fix#P8 (v10.5.5):** Partial integrator di dalam deadband dengan gain 10%:
  ```cpp
  float e_db = constrain(e_raw, -1.0f, 1.0f);
  pidInteg += (pidKi * 0.1f * e_db * dt);  // 10% gain, pakai e_raw bukan e=0
  ```
  Cukup untuk kompensasi outflow konstan, tidak cukup agresif untuk hunting.

#### Behavior setelah fix
| Kondisi | Sebelum v10.5.5 | Setelah v10.5.5 |
|---------|----------------|----------------|
| PV settle di deadband | Bisa ±2.5% dari SP | Maksimal ±1.0% dari SP |
| Outflow saat deadband | Drift turun pelan | Terkompensasi partial integrator |
| PV > SP di deadband | Decay 0.9999 (sangat lambat) | Partial integrator tarik turun |
| Hunting | Tidak ada | Tidak ada (gain 10% cukup halus) |

---

## Observer Dashboard (v10 — aquaflow_observer_v10.html)

### Masalah: Observer mati tiap beberapa detik + tab switch

#### FIX#AT1 — Autotune shadow sim masih ZN Classic
- Shadow sim di observer pakai formula `0.45*Ku` (ZN Classic)
- Hasil rekomendasi Kp/Ki tidak konsisten dengan pkk.ino
- **Fix:** Ganti ke Tyreus-Luyben + de-tune 0.6x: `Kp = 0.190*Ku`, `Ki = Kp/(2.2*Pu)`

#### FIX#OB1 — Observer buffer stale setelah reconnect
- `initObserver()` hanya dipanggil kalau `!plantId` → saat reconnect, buffer tidak di-reset
- Shadow sim bisa desync setelah disconnect/reconnect
- **Fix:** `initObserver()` dipanggil setiap `ws.onopen` tanpa kondisi

#### FIX#OB2 — Observer mati saat pindah ke tab dashboard
- Tidak ada `visibilitychange` handler → saat tab di-background, browser throttle `setInterval`
- Ping timer freeze → Railway proxy timeout 12 detik → WS putus → observer mati
- **Fix:** Tambah `visibilitychange` listener — reconnect jika WS putus, restart ping jika masih hidup

---

## WebSocket Keepalive (Railway/Cloud Proxy)

Railway dan cloud proxy lainnya memiliki **idle timeout ~10-12 detik** untuk koneksi WebSocket.
Kalau tidak ada WebSocket ping frame dalam waktu tersebut, proxy memutus koneksi tanpa notifikasi.

> ⚠️ Data JSON (`{type:'state', ...}`) **tidak dihitung** sebagai keepalive oleh proxy.
> Harus menggunakan WebSocket native ping frame.

**Fix di v10.5.3+ (FIX#S3):**
- `server.js`: `client.ping()` ke semua client setiap 8 detik
- `aquaflow_observer_v10.html`: `ws.send({type:'ping'})` dari browser setiap 8 detik

---

## Changelog

| Versi | Fix | Deskripsi |
|-------|-----|-----------|
| v10.5.5 | FIX#P7, FIX#P8 | Deadband 2.5→1.0%, partial integrator 10% di deadband |
| v10.5.4 | FIX#P6 | ZN Classic → Tyreus-Luyben + de-tune 0.6x |
| v10.5.3 | FIX#S3 | WebSocket keepalive ping/pong |
| v10.5.2 | FIX#P3, FIX#P4 | Deadband freeze, relayD fix |
| v10 observer | FIX#AT1, FIX#OB1, FIX#OB2 | Shadow sim TL formula, observer reinit, visibilitychange |

