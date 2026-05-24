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

## WebSocket Keepalive (Railway/Cloud Proxy)

Railway dan cloud proxy lainnya memiliki **idle timeout ~10-12 detik** untuk koneksi WebSocket.
Kalau tidak ada WebSocket ping frame dalam waktu tersebut, proxy memutus koneksi tanpa notifikasi.

> ⚠️ Data JSON (`{type:'state', ...}`) **tidak dihitung** sebagai keepalive oleh proxy.
> Harus menggunakan WebSocket native ping frame.

**Fix di v10.5.3+ (FIX#S3):**
- `server.js`: `client.ping()` ke semua client setiap 8 detik
- `aquaflow_observer_v10.html`: `ws.send({type:'ping'})` dari browser setiap 8 detik

