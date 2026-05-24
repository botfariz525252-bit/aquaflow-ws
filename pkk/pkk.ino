//  *** VERSION v10.6.1 — FIX#P13: pompa tidak mati saat PV>SP (PWM floor salah kondisi) ***
//
//  FIX#P13 — Output pompa tidak benar-benar 0 saat PV sudah melewati SP
//    - Bug A: setPWM() floor PWM_MIN_STEADY aktif juga saat PV>SP (e_raw<0)
//      Akibat: pompa masih jalan pelan → PV terus naik walaupun sudah lewat SP
//    - Bug B: pidStep() paksa o=1 saat error<5% tanpa cek arah error
//      Akibat: saat e_raw<0 (PV>SP), output tetap 1% → pompa tidak pernah 0
//    - Fix A: kondisi floor di setPWM() tambah cek e_raw — hanya aktif saat PV<SP
//    - Fix B: kondisi paksa o=1 di pidStep() tambah e_raw>0 guard
//
//  *** VERSION v10.6.0 — FIX#P11: validasi PV range anti-garbage, FIX#P12: pompa gradual start ***
//
//  FIX#P11 — LCD tampil PV random (misal 102090930)
//    - Bug: serial buffer corruption / partial frame → atof() baca sampah → PV gila
//    - Fix: validasi output readPV() — kalau pv < -5 atau pv > 110, reject → pakai lastPV
//    - Tambah counter pvRejectCount untuk debug
//
//  FIX#P12 — Pompa kenceng banget saat 1% steady state
//    - Bug: pidSettledOnce=0 saat observer baru buka → pompa mulai dari PWM_MIN_RUN=50
//      langsung → PV overshoot naik cepat
//    - Fix: saat pidSettledOnce=0 tapi pidInteg > 0 (ada riwayat), set settled=1
//      supaya floor PWM_MIN_STEADY=25 yang dipakai, bukan langsung 50
//
//  *** VERSION v10.5.9 — FIX#P10: antiwindup di deadband, cegah PV drift naik perlahan ***
//
//  FIX#P10 — Integrator drift naik saat steady state
//    - Bug: partial integrator (FIX#P8) + PWM floor (FIX#P9) = integrator windup pelan
//      Di dalam deadband: antiwindup TIDAK aktif (hanya di luar deadband)
//      Akibat: pidInteg akumulasi terus → output naik pelan → PV lama-lama jauh di atas SP
//    - Fix 1: di dalam deadband, kalau e_raw < 0 (PV sudah di atas SP), DECAY integrator
//      pakai faktor 0.999 per step — pelan tapi pasti turun
//    - Fix 2: terapkan antiwindup (wErr) juga di dalam deadband
//      supaya kalau output sudah saturasi, integrator tidak terus naik
//  *** VERSION v10.5.8 — FIX#P9: steady-state PWM floor, cegah pompa nyala-mati ***
//
//  FIX#P9 — PWM floor saat steady state: cegah hunting pompa nyala-mati
//    - Bug: saat PV sudah dekat SP (steady state), output PID bisa turun ke 0
//      karena partial integrator terlalu lemah kompensasi outflow
//      Akibat: setPWM(0) → pompa mati → PV turun → PID nyalain lagi → PWM_MIN_RUN=50
//      Pompa langsung kenceng (1% sudah lumayan kenceng) → PV overshoot → mati lagi → hunting
//    - Fix: tambah PWM_MIN_STEADY=25 sebagai floor output saat pidSettledOnce=1
//      Output PID tidak pernah 0 saat steady state → pompa selalu jalan pelan
//      Hanya berlaku saat error kecil (|e|<5%), kalau error besar tetap bisa 0
//      Pompa nyala terus tapi pelan → level smooth, tidak ada nyala-mati
//  *** VERSION v10.5.7 — FIX#AB1: auto-bias estimation sebelum relay, bias optimal dari dPV ***
//  *** VERSION v10.5.6 — FIX#RAM1-3: hemat RAM — shBuf 60→56, PSTR Serial format, F() LCD ***
//  *** VERSION v10.5.5 — FIX#P7+P8: deadband 2.5→1.0, partial integrator di deadband ***
//
//  FIX#P6 — ZN formula: ganti ZN Classic → Tyreus-Luyben + de-tune 0.6x
//    - Bug: ZN Classic (Kp=0.45*Ku PI, Kp=0.6*Ku PID) terlalu agresif untuk sistem
//      tangki asimetris (naik cepat via pompa, turun lambat via outflow gravitasi)
//    - Akibat: overshoot, PV nyangkut 3-4% DI ATAS SP, steady-state error positif
//    - Fix: kombinasi dua pendekatan:
//      1. Tyreus-Luyben formula (lebih konservatif untuk proses lambat):
//           PI:  Kp = 0.3158*Ku,  Ti = 2.2*Pu  → Ki = Kp/Ti
//           PID: Kp = 0.4545*Ku,  Ti = 2.2*Pu,  Td = Pu/6.3 → Ki = Kp/Ti, Kd = Kp*Td
//      2. De-tune 0.6x: Kp dan Ki dikalikan 0.6 setelah TL → overall lebih smooth
//    - Result: rise time sedikit lebih lambat tapi ZERO steady-state error, no overshoot
//
//  FIX#P5 — Deadband integrator: freeze saat PV<SP (e_raw>0), decay ultra-pelan saat PV>SP
//    - Bug v10.5.2: integrator decay 0.998 di SELURUH deadband
//      Akibat: saat PV mendekati SP dari bawah dan masuk deadband,
//      integrator langsung meleleh → pompa output turun → PV tidak bisa hold di SP
//      → steady-state error 3-4% di bawah SP (PV tidak bisa reach SP)
//    - Fix: split deadband behavior berdasarkan arah error:
//        e_raw > 0 (PV < SP): FREEZE integrator (jaga nilai, jangan turunkan)
//        e_raw < 0 (PV > SP): decay ultra-pelan 0.9999 (biarkan pompa alami turun sedikit)
//        e_raw = 0 (tepat SP): freeze
//    - Dengan fix ini: integrator menyimpan nilai yang diperlukan untuk kompensasi outflow
//      sehingga PV dapat bertahan tepat di SP meski ada outflow konstan
//
//  *** VERSION v10.5.2 — FIX: PWM_MIN, deadband freeze, relayD, observer timeout ***
//
//  FIX#P1 — PWM_MIN_RUN 80→50: pompa 5.5LPM tangki ~27L, min PWM lebih rendah
//  FIX#P2 — Deadband 1.5→2.5%: mencegah pompa nyala terus saat dekat SP
//  FIX#P3 — Freeze integrator di deadband: ganti creep 0.05 → decay 0.998
//  FIX#P4 — relayD=TUN_AMP: Ku/Kp/Ki autotune dihitung akurat
//
//  *** VERSION v10.5 — FIX: Kalibrasi berulang + alarm EN + EE_MAGIC bump 0xB4 ***
//
//  FIX#94~98 — SEMUA eeMarkDirty() standalone → tambah eeForceFlush() LANGSUNG
//    - FIX#94: auto-tune selesai → save Kp/Ki/Kd/tunDone langsung
//    - FIX#95: ESP set SP → save langsung  
//    - FIX#96~98: ESP set Kp/Ki/Kd → save langsung
//    - ROOT CAUSE FINAL: tidak ada lagi aksi user yang hanya eeMarkDirty tanpa flush
//
//  FIX#92 — goBack() dari S_CALV: save sLRV saat back
//    - Bug: user tekan B di S_CALV (update sLRV), lalu tekan A (back ke S_CAL)
//      sLRV berubah tapi TIDAK disimpan ke EEPROM
//    - Fix: eeForceFlush() saat goBack dari S_CALV
//
//  FIX#93 — goBack() dari S_CALD: pastikan tersimpan saat keluar
//    - S_CALD = layar konfirmasi selesai kalibrasi
//    - Fix: eeForceFlush() saat goBack dari S_CALD (double safety)
//
//  FIX#91 — EE_MAGIC bump 0xB3 → 0xB4 (WAJIB untuk fix alarm save)
//    - Root cause alarm tidak tersimpan:
//      Versi lama (v9.x) punya EE_MIN=60 detik → user power off sebelum 60 detik → HILANG
//      User upload v10.x → magic mismatch (0xB1→0xB3) → flush defaults (alarm OFF=0)
//      Sisa EEPROM bisa korup atau punya nilai lama yang salah offsetnya
//    - Dengan bump ke 0xB4: SEMUA user dapat fresh start
//      Boot pertama setelah upgrade: magic mismatch → defaults ditulis (alarm OFF)
//      User set alarm → commitEdit() eeForceFlush() → LANGSUNG TERSIMPAN
//      Restart → magic 0xB4 match → load benar
//
//
//  FIX#85 — S_CAL pilih LRV (cur==0): reset urvSetDone=0
//    - Bug: kalibrasi ke-2+: urvSetDone masih=1 dari kalibrasi sebelumnya
//      FIX#78 auto-set sURV=sRAW saat '#' di S_CALV (posisi tangki kosong)
//      → sURV = sLRV = tangki kosong → span=0 → ERR:LRV<=URV!
//    - Fix: saat user pilih SET LRV di S_CAL, reset urvSetDone=0
//
//  FIX#86 — S_CALV '#' → S_CALU: BATALKAN FIX#78 auto-set sURV=sRAW
//    - Bug root dari FIX#78: auto-set sURV saat '#' dari S_CALV rusak kalibrasi ke-2+
//      karena sRAW di S_CALV = posisi tangki kosong (baru saja jadi LRV)
//    - Fix: hapus auto-set, biarkan user harus pencet 'B' di posisi tangki penuh
//      urvSetDone=0 → LCD tampil "ISI PENUH B=URV" sebagai panduan
//
//  FIX#87 — dCalSub(LRV): label LCD "B=LRV #=Next"
//    - Fix tampilan instruksi lebih jelas
//
//  FIX#88 — dCalSub(URV): label LCD "ISI PENUH B=URV" kalau belum di-set
//    - Panduan: user harus isi tangki ke posisi penuh dulu, lalu pencet B
//
//  FIX#89 — goBack() dari S_CALU: eeMarkDirty → eeForceFlush
//    - Bug FIX#74: save URV saat back hanya pakai eeMarkDirty → bisa hilang
//    - Fix: eeForceFlush() → langsung tulis ke EEPROM
//
//  FIX#90 — readFromESP(): ahen/alen command → eeForceFlush immediate
//    - Bug: ESP kirim toggle alarm → eeMarkDirty saja → bisa hilang kalau power off cepat
//    - Fix: tambahkan eeForceFlush() setelah eeMarkDirty()
//
//  *** VERSION v10.4 — FIX: EEPROM magic mismatch langsung flush defaults ***
//
//  FIX#84 — eeLoad(): magic mismatch → eeForceFlush() bukan eeMarkDirty()
//    - Bug utama: saat magic tidak cocok (setelah EE_MAGIC bump atau EEPROM kosong),
//      kode lama cuma panggil eeMarkDirty() → tunggu 5 detik → eeFlush()
//      Kalau user reset sebelum 5 detik → EEPROM masih invalid → loop terus
//    - Akibat: settings tidak pernah benar-benar tersimpan karena setiap boot
//      magic mismatch lagi → reset ke defaults → mismatch → dst
//    - Fix: langsung eeForceFlush() saat magic mismatch
//      Defaults langsung ditulis ke EEPROM di boot pertama
//      Boot berikutnya: magic match → load values benar
//

//  *** VERSION v10.4 — FIX: EEPROM magic mismatch langsung flush defaults ***
//
//  FIX#83 — commitEdit(): eeMarkDirty() → eeForceFlush()
//    - Bug ROOT: setiap kali user input nilai (alarm Hi, alarm Lo, SP, Kp, dll)
//      commitEdit() cuma panggil eeMarkDirty() → tunggu 5 detik → eeFlush()
//      Kalau reset/power off sebelum 5 detik → SEMUA input HILANG
//    - Bug ini kenapa alarm Hi/Lo tidak ke-save: user input → commitEdit
//      → eeMarkDirty saja → reset → hilang
//    - Fix: ganti eeMarkDirty() → eeForceFlush() di akhir commitEdit()
//      Sekarang SEMUA input (SP, Kp, Ki, Kd, alarm Hi, alarm Lo, LRV, URV manual)
//      langsung ditulis ke EEPROM setelah user konfirmasi (#)
//

//  *** VERSION v10.3 — FIX: semua input setting langsung flush EEPROM ***
//
//  FIX#78 — ROOT BUG: sURV tidak di-set saat '#' di S_CALV → S_CALU
//    - Bug lama tersembunyi: saat '#' di layar S_CALV (LRV done, next),
//      kode hanya: scr=S_CALU tanpa set sURV=sRAW
//    - Akibat: kalau user di S_CALU tidak pencet 'B' dulu lalu langsung '#',
//      sURV = nilai lama/default (5cm) → sLRV-sURV span salah
//    - readPV() pakai span salah → PV tidak naik/selalu 0 setelah kalibrasi!
//    - Fix: saat '#' di S_CALV, langsung set sURV=sRAW sebelum pindah ke S_CALU
//      Jadi sensor otomatis ter-set dari posisi sekarang (posisi tangki penuh)
//
//  FIX#79 — urvSetDone di-set 1 sekaligus saat pre-set di S_CALV
//    - Konsistensi: kalau sURV sudah valid, urvSetDone harus 1
//
//  FIX#80 — S_CAL jalur langsung (cur==1): urvSetDone=1 langsung
//    - Dulu: urvSetDone=0 → LCD tampil "B=Set" padahal sURV sudah di-set
//    - Fix: urvSetDone=1 saat masuk S_CALU dari jalur langsung S_CAL
//

//  *** VERSION v10.1 — BUG FIX: URV tidak ter-set saat kalibrasi (root bug PV=0) ***
//
//  FIX#73 — URV kalibrasi freeze silent → tampil error "ERR:LRV<=URV!"
//    - Bug: kalau '#' dipencet saat LRV <= URV+1, layar diam tidak ada feedback
//    - User tidak tahu kenapa tidak bisa done → ngerasa freeze
//    - Fix: tampilkan "ERR:LRV<=URV! B=Set" di baris ke-4 LCD
//
//  FIX#74 — Back dari S_CALU: URV yang sudah di-set (B) tidak disimpan EEPROM
//    - Bug: user pencet B (URV=sRAW) lalu back → eeMarkDirty tidak dipanggil
//    - Fix: kalau urvSetDone=1 saat back dari S_CALU, panggil eeMarkDirty()
//
//  FIX#75 — Ganti unit: prevSP tidak di-reset → FF feedforward salah sesaat
//    - Fix: prevSP=pidSP saat unitMode berubah
//
//  FIX#76 — S_CALU LCD: tampilkan LRV vs URV saat sedang set URV
//    - Bug: user tidak tahu nilai LRV saat set URV → tidak bisa debug sendiri
//    - Fix: baris ke-3 LCD tampilkan "LRV:xx.xcm URV:xx.xcm"
//    - Bonus: kalau LRV<=URV, baris ke-4 tampilkan warning "!LRV<=URV B=ReSet"
//
//  FIX#77 — eeForceFlush: simpan EEPROM langsung tanpa tunggu EE_MIN
//    - Bug: kalibrasi selesai (S_CALD) → eeMarkDirty dipanggil TAPI
//      eeFlush masih tunggu 5 detik → kalau reset cepat setelah kalibrasi → hilang
//    - Juga: toggle alarm ON/OFF dan set unit tidak segera disimpan
//    - Fix: tambah eeForceFlush() yang skip EE_MIN check
//      Dipanggil saat: kalibrasi selesai, toggle alarm, ganti unit
//

//  *** VERSION v10.0 — BUG FIX: URV freeze + units save + alarm EEPROM + force flush ***
//
//  FIX#67 — EE_MIN dikurangi dari 60000ms → 5000ms
//    - Bug: user toggle alarm OFF → eeMarkDirty() terpanggil ✓
//      Tapi kalau power off/reset sebelum 60 detik → setting TIDAK tersimpan ke EEPROM!
//    - Fix: EE_MIN = 5000ms (5 detik), masih aman untuk EEPROM UNO (~100k write cycles)
//
//  FIX#68 — Default alarm di setup(): alarmHiEn=0, alarmLoEn=0 (OFF)
//    - Bug: default alarm ON dengan threshold Hi=90, Lo=10
//      Saat tangki kosong boot: PV=0 < alarmLo=10 → alarmLoAct=true
//      Setelah pidSettledOnce=1 (sesaat), pompa langsung di-STOP oleh alarmLo
//    - Fix: default alarm OFF, user aktifkan manual setelah kalibrasi sensor
//
//  FIX#69 — F struct init: alarmLoAct=0, alarmHiEn=0, alarmLoEn=0, tunReasonIdx=0
//    - Bug: F struct init = {1,0,0,0,0,0,0,0,1,1,0,0,1,0}
//      alarmLoAct=1 (!), alarmHiEn=1, tunReasonIdx=1 saat startup sebelum eeLoad()
//    - Fix: semua alarm flag = 0 saat init, tunReasonIdx = 0
//
//  FIX#70 — Hapus double readFromESP() di loop()
//    - Bug: readFromESP() dipanggil 2x berturut-turut di loop()
//      Kalau 2 baris serial datang bersamaan, baris ke-2 bisa diproses
//      sebelum UI update → potensi jitter pada display dan state
//    - Fix: satu kali panggilan readFromESP() per loop cycle
//
//  FIX#71 — F struct init alarmLoAct reset ke 0 (bagian dari FIX#69)
//
//  FIX#72 — EE_MAGIC bump 0xB1 → 0xB2
//    - Paksa re-init EEPROM dengan default baru (alarm OFF)
//    - EEPROM lama dengan alarm ON akan diabaikan dan ditimpa defaults v9.9
//

//  *** VERSION v10.0 — BUG FIX: URV freeze + units save + alarm EEPROM + force flush ***
//
//  CHANGES v9.8 (dari v9.7):
//
//  FIX#63 — pvPrev & pvRate tidak di-reset saat startTune() dan cancelTune()
//    - Bug: setelah autotune selesai, pidStep pertama hitung pvRate dari pvPrev
//      yg stale (nilai sebelum autotune mulai, bisa beda 10-30% dari PV sekarang)
//    - Akibat: pvRate spike → rate correction aktif salah → pump surge sesaat
//    - Fix: tambahkan pvPrev=lastPV; pvRate=0; di startTune() & cancelTune()
//
//  FIX#64 — pidPrevPV double-assignment di loop()
//    - Bug: pidPrevPV di-assign di pidStep() lalu di-assign lagi di loop()
//    - Efek: saat tuneStep aktif, pidPrevPV di-update terus tidak perlu
//    - Fix: hapus pidPrevPV=pv dari loop(), biarkan pidStep() yang handle
//
//  FIX#65 — alarmLo tidak aktif saat autotune cancel sebelum pidSettledOnce
//    - Bug: alarmLo disable total kalau pidSettledOnce=0, termasuk saat tunCancel
//    - Fix: alarmLo aktif juga saat F.tunCancel, bukan hanya F.pidSettledOnce
//
// ============================================================
//  AquaFlow_UNO.ino — PID Water Level Controller
//  *** VERSION v9.8 — BUG FIX: pvPrev stale + alarmLo boot + pvPrevPV double ***
//
//  CHANGES v9.8 (dari v9.7):
//
//  FIX#63 — pvPrev & pvRate tidak di-reset saat startTune() dan cancelTune()
//    - Bug: setelah autotune selesai, pidStep pertama hitung pvRate dari pvPrev
//      yang stale (nilai sebelum autotune, bisa beda 10-30% dari PV sekarang)
//    - Akibat: pvRate spike besar → rate correction salah → pump surge sesaat
//    - Fix: tambahkan pvPrev=lastPV; pvRate=0; di startTune() dan cancelTune()
//
//  FIX#64 — pidPrevPV double-assignment di loop()
//    - Bug: pidPrevPV di-assign di pidStep() baris akhir, lalu di-assign LAGI
//      di loop() setelah pidStep()/tuneStep() dengan nilai yang sama
//    - Efek: benign saat PID jalan (nilai sama), TAPI saat tuneStep aktif
//      pidPrevPV di-update terus oleh loop() padahal tidak perlu
//    - Fix: hapus pidPrevPV=pv; dari loop(), biarkan pidStep() yang handle
//
//  FIX#65 — alarmLo tidak aktif saat boot sebelum pidSettledOnce
//    - Bug: kondisi alarmLo hanya cek F.pidSettledOnce untuk hindari false alarm
//      saat startup. Tapi ini juga disable alarmLo selama autotune fill/drain!
//    - Skenario: tangki hampir kosong (PV < alarmLo=10%) → autotune dimulai →
//      pompa fill terus (benar), tapi kalau autotune cancel, pompa tetap jalan
//      tanpa alarm lo karena pidSettledOnce=0. 
//    - Fix: alarmLo tetap skip saat startup biasa (pvStart<alarmLo normal),
//      tapi aktif saat tunCancel. Tambahkan cek F.tunCancel.
//
//  CHANGES v9.7 (dari v9.6):
//
//  
//
//  FIX#61 — URV Kalibrasi '#' tidak berfungsi
//    - Kondisi lama: sLRV > sURV + 5  (gagal kalau selisih < 5cm)
//    - Fix: sLRV > sURV + 1.0f
//    - Tambah visual feedback "URV SET" di LCD baris ke-3
//
//  FIX#62 — Konfirmasi overshoot = 0% (verified by simulation)
//    - "Overshoot 1.5%" yang terlihat sebelumnya bukan overshoot sejati
//    - PV tidak pernah melampaui SP+1.5% (batas deadband atas)
//    - Yang terjadi: PV settling di dalam deadband ±1.5% = NORMAL
//    - Steady state: PV ≈ SP ± 0.5% (stabil sangat baik)
//    - Nilai Kp/Ki dari autotune sudah optimal, TIDAK perlu de-tune
//
//  NILAI TUNING (dari simulasi sistem Nikuma 5LPM + Tangki 10L):
//    PI mode:  Kp=14.09  Ki=1.04
//    PID mode: Kp=18.79  Ki=2.31  Kd=2.03
//
// ============================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>

#define PIN_TRIG 3
#define PIN_ECHO 2
#define PIN_PUMP 9

LiquidCrystal_I2C lcd(0x27, 20, 4);

const byte ROWS=4, COLS=4;
char keys[ROWS][COLS]={
  {'1','2','3','A'},{'4','5','6','B'},
  {'7','8','9','C'},{'*','0','#','D'}
};
byte rp[ROWS]={A0,A1,A2,A3};
byte cp[COLS]={5,6,7,8};
Keypad kpd=Keypad(makeKeymap(keys),rp,cp,ROWS,COLS);

enum UnitMode:uint8_t{U_PCT,U_CM,U_M,U_MMH};
enum AMode   :uint8_t{AM_MANUAL,AM_PID};
enum TMode   :uint8_t{TM_PI,TM_PID};
enum Scr     :uint8_t{
  S_SPLASH,S_STAT,S_MENU,S_MODE,S_PSUB,S_ARUN,
  S_MTUN,S_MPWM,S_CAL,S_CALV,S_CALU,
  S_CALD,S_UNIT,S_EDIT,S_ALARM,S_IP
};
enum ET:uint8_t{
  E_NO,E_SP,E_KP,E_KI,E_KD,
  E_LRV,E_URV,E_MPWM,E_ALHI,E_ALLO,E_FF
};

UnitMode unitMode=U_PCT;
AMode amode=AM_PID;
TMode tmode=TM_PI;
Scr scr=S_STAT, eback=S_STAT;
ET etgt=E_NO;
uint8_t cur=0,mscr=0,unitScr=0;
uint8_t pidLastPWM=0,manPWM=0;
// FIX#60: tunNeed 8→4 (cukup 2 siklus penuh untuk akurasi relay method)
uint8_t tunNeed=4,tunXcount=0,sensorErrCount=0;

struct Flags {
  uint8_t pidOn         :1;
  uint8_t eeDirty       :1;
  uint8_t tunActive     :1;
  uint8_t tunDone       :1;
  uint8_t tunCancel     :1;
  uint8_t kickActive    :1;
  uint8_t pidSettledOnce:1;
  uint8_t alarmHiAct    :1;
  uint8_t alarmLoAct    :1;
  uint8_t alarmHiEn     :1;
  uint8_t alarmLoEn     :1;
  uint8_t mfill         :1;
  uint8_t tunReasonIdx  :2;
  uint8_t tunFilling    :1;
  uint8_t tunDraining   :1;
  // FIX#60: flag baru — skip fill/drain langsung relay
  uint8_t tunDirectRelay:1;
  // FIX#61: flag URV sudah di-set via tombol B
  uint8_t urvSetDone    :1;
  // FIX#AB1: flag fase auto-bias estimation
  uint8_t tunBiasEst    :1;
} F = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  // FIX#AB1: tambah tunBiasEst=0

const char tr0[] PROGMEM="NONE";
const char tr1[] PROGMEM="USER";
const char tr2[] PROGMEM="TIMEOUT";
const char* const trT[] PROGMEM={tr0,tr1,tr2};

float sLRV=50,sURV=5,sRAW=0;
float pidSP=50,pidKp=5,pidKi=0.2f,pidKd=0;
float pidKff=0.5f,prevSP=50;
float pidInteg=0,pidPrevPV=0,dFiltered=0;
float pvRate=0,pvPrev=0;
float alarmHi=90,alarmLo=10;

#define TUN_BIAS 50.0f
#define TUN_AMP  50.0f
float tunPVhi,tunPVlo,tunSign=0;
// FIX#AB1: auto-bias estimation variables
float tunBiasEstUp=0,tunBiasEstDown=0; // rate naik/turun saat estimasi (%/detik)
float tunBiasEstPVstart=0;             // PV saat fase estimasi dimulai
uint32_t tunBiasEstMs=0;               // timestamp mulai tiap fase
uint8_t tunBiasEstPhase=0;            // 0=idle,1=ukur naik(pompa ON),2=ukur turun(pompa OFF),3=selesai
float tunBiasDynamic=TUN_BIAS;        // bias yang dihitung dari estimasi
float emaVal=0,lastPV=0,pvS=0,lastPVdisp=0;
bool  emaInitDone=false;

uint32_t tCtrl=0,tLCD=0,eeLastMs=0,espLastMs=0;
uint8_t bootWarmup=0;
uint32_t tunLastMs=0,tunPrevMs=0;
uint32_t kickStartMs=0;
uint32_t tunFillStartMs=0;
uint32_t tunDrainStartMs=0;

#define KICKSTART_PWM       160
#define KICKSTART_MS        700UL
#define PWM_MIN_RUN         50    // FIX#P1: turunkan min PWM, pompa 5.5LPM tangki ~27L
#define PWM_MIN_STEADY      25    // FIX#P9: floor PWM saat steady state (pompa tetap pelan, tidak mati)
#define PWM_MAX_RUN         250
#define ESP_INTERVAL        500UL
#define MED_N               3
#define EMA_A               0.15f
#define Ts_MS               100UL
#define EE_MIN              5000UL   // FIX#67: was 60000UL, 5s cukup aman (UNO EEPROM ~100k writes)
#define SENSOR_ERR_MAX      10
#define ANTIWINDUP_KC       1.2f
#define KICKSTART_ERR_MIN   20.0f
#define INTEG_SP_RESET_THR  15.0f
#define D_FILTER_A          0.05f
#define EE_MAGIC            0xB4     // FIX#91 v10.5: bump — paksa re-init, fix alarm EEPROM save definitif

#define DT_MAX              0.20f

// FIX#60: timeout fill/drain saat startTune disingkat ke 5 menit
// (dulu FILL=5min, DRAIN=10min, sekarang sama-sama 5min maksimal)
#define TUN_FILL_TIMEOUT_MS        300000UL   // 5 menit
#define TUN_DRAIN_TIMEOUT_MS       300000UL   // 5 menit (FIX#60: dikurangi dari 10 menit)
#define TUN_FIRST_CROSS_TIMEOUT_MS 600000UL
#define TUN_HALF_CYCLE_TIMEOUT_MS  600000UL
#define RELAY_BAND          2.0f

// FIX#60: threshold jarak PV-SP untuk skip fill/drain langsung relay
#define TUN_DIRECT_RELAY_THR  22.0f  // kalau |PV-SP| <= 22% → langsung relay

float mbuf[MED_N];
uint8_t midx=0;

volatile uint32_t echoStart    = 0;
volatile uint32_t echoDuration = 0;
volatile bool     echoReady    = false;

static char shBuf[56];  // FIX#RAM1: 60->56, cukup untuk string terpanjang 54 chars
static char lbBuf[21];
#define lb lbBuf

static char ta[8], tb[8];
static char nbuf[9];

#define RXBUF_SIZE 72
char rxBuf[RXBUF_SIZE];
uint8_t rxIdx=0;

uint8_t lastKeyPWM=0;
uint32_t keyPressTime=0;

// =============================================
int pwmToDisplay(uint8_t pwm){
  if(pwm==0) return 0;
  return (int)constrain(map((long)pwm,(long)PWM_MIN_RUN,(long)PWM_MAX_RUN,0L,100L),0L,100L);
}

void echoISR(){
  if(digitalRead(PIN_ECHO)==HIGH){
    echoStart=micros();
  } else {
    if(echoStart>0){
      echoDuration=micros()-echoStart;
      echoReady=true;
      echoStart=0;
    }
  }
}

void sonarUpdate(){
  static uint32_t lastTrigMs=0;
  uint32_t now=millis();
  if(now-lastTrigMs<60UL) return;
  lastTrigMs=now;
  digitalWrite(PIN_TRIG,LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG,HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG,LOW);
}

float readDist(){
  if(!echoReady) return NAN;
  noInterrupts();
  uint32_t dur=echoDuration;
  echoReady=false;
  interrupts();
  float d=(float)dur*0.0343f/2.0f;
  if(d<2.0f||d>400.0f) return NAN;
  sRAW=d;
  return d;
}

struct __attribute__((packed)) EERec{
  uint8_t mag,unit;
  float sp,kp,ki,kd,lrv,urv,alHi,alLo,kff;
  uint8_t alHiEn,alLoEn;
};
void eeMarkDirty(){F.eeDirty=1;}
void eeForceFlush(){  // FIX#77: flush segera tanpa tunggu EE_MIN (untuk kalibrasi & alarm)
  if(!F.eeDirty) return;
  EERec d={EE_MAGIC,(uint8_t)unitMode,
           pidSP,pidKp,pidKi,pidKd,sLRV,sURV,alarmHi,alarmLo,pidKff,
           F.alarmHiEn?1:0,F.alarmLoEn?1:0};
  EEPROM.put(0,d); eeLastMs=millis(); F.eeDirty=0;
}
void eeFlush(uint32_t now){
  if(!F.eeDirty||now-eeLastMs<EE_MIN)return;
  EERec d={EE_MAGIC,(uint8_t)unitMode,
           pidSP,pidKp,pidKi,pidKd,sLRV,sURV,alarmHi,alarmLo,pidKff,
           F.alarmHiEn?1:0,F.alarmLoEn?1:0};
  EEPROM.put(0,d); eeLastMs=now; F.eeDirty=0;
}
void eeLoad(){
  EERec d; EEPROM.get(0,d);
  if(d.mag!=EE_MAGIC){
    // FIX#84: magic mismatch → tulis defaults langsung ke EEPROM sekarang
    // Bukan eeMarkDirty (tunggu 5 detik) tapi langsung EEPROM.put
    // Ini pastikan setelah magic bump, EEPROM valid dari pertama boot
    F.eeDirty=1;  // set dirty dulu supaya eeForceFlush mau jalan
    eeForceFlush();
    return;
  }
  unitMode=(UnitMode)constrain((int)d.unit,0,3);
  pidSP=constrain(d.sp,0,100);   pidKp=constrain(d.kp,0,500);
  pidKi=constrain(d.ki,0,500);   pidKd=constrain(d.kd,0,500);
  sLRV=constrain(d.lrv,0,400);   sURV=constrain(d.urv,0,400);  // FIX#81: batas 200→400 (max range HC-SR04)
  alarmHi=constrain(d.alHi,0,100); alarmLo=constrain(d.alLo,0,100);
  pidKff=constrain(d.kff,0,10);
  F.alarmHiEn=(d.alHiEn!=0); F.alarmLoEn=(d.alLoEn!=0);
}

static inline float cf(float x,float a,float b){return x<a?a:x>b?b:x;}
static inline int   ci(int   x,int   a,int   b){return x<a?a:x>b?b:x;}

void LP(uint8_t r,const char* s){
  lcd.setCursor(0,r);uint8_t i=0;
  while(i<20&&s[i])lcd.write(s[i++]);
  while(i++<20)lcd.write(' ');
}
void LP_P(uint8_t r,const char* ps){
  lcd.setCursor(0,r);uint8_t i=0;char c;
  while(i<20&&(c=pgm_read_byte(ps+i++)))lcd.write(c);
  while(i++<20)lcd.write(' ');
}

const char ul0[] PROGMEM="% ";
const char ul1[] PROGMEM="cm ";
const char ul2[] PROGMEM="m ";
const char ul3[] PROGMEM="mmH";
const char* const ulT[] PROGMEM={ul0,ul1,ul2,ul3};

const char* getUL(){
  static char b[5];
  strcpy_P(b,(const char*)pgm_read_ptr(&ulT[unitMode]));
  return b;
}
void fmtVal(float v,char* buf){
  if(unitMode==U_M){dtostrf(v,4,2,buf);buf[4]=0;}
  else snprintf_P(buf,5,PSTR("%4d"),(int)(v+0.5f));
}
void stopPump(){
  analogWrite(PIN_PUMP,0);
  pidLastPWM=0;
  F.kickActive=0;
}
float toUnit(float p){
  float sp=sLRV-sURV;if(sp<0.01f)sp=0.01f;
  float cm=p*sp/100.0f;
  switch(unitMode){
    case U_PCT:return p; case U_CM:return cm;
    case U_M:return cm/100.0f; case U_MMH:return cm*10.0f;
  }
  return p;
}
float fromUnit(float u){
  float sp=sLRV-sURV; if(sp<0.01f)sp=0.01f;
  float cm;
  switch(unitMode){
    case U_PCT:  return cf(u,0,100);
    case U_CM:   cm=u; break;
    case U_M:    cm=u*100.0f; break;
    case U_MMH:  cm=u/10.0f; break;
    default:     return cf(u,0,100);
  }
  return cf(cm*100.0f/sp,0,100);
}

void resetSensorFilter(){
  emaVal = lastPV;
  pvS    = lastPV;
  emaInitDone = true;
  midx   = 0;
  F.mfill = 0;
  memset(mbuf, 0, sizeof(mbuf));
}

void setPWM(int pw){
  if(pw<1){
    analogWrite(PIN_PUMP,0); pidLastPWM=0; F.kickActive=0; return;
  }
  if(pidLastPWM==0&&!F.kickActive){
    float errNow = fabs(pidSP - lastPV);
    if(errNow >= KICKSTART_ERR_MIN){
      F.kickActive=1; kickStartMs=millis();
      analogWrite(PIN_PUMP,KICKSTART_PWM); pidLastPWM=KICKSTART_PWM; return;
    }
    F.kickActive=0;
  }
  if(F.kickActive){
    if(millis()-kickStartMs<KICKSTART_MS){
      analogWrite(PIN_PUMP,KICKSTART_PWM); pidLastPWM=KICKSTART_PWM; return;
    }
    F.kickActive=0; pidLastPWM=PWM_MIN_RUN;
    pidInteg=0; dFiltered=0;
  }
  int pwM=(int)map((long)pw,0,100,(long)PWM_MIN_RUN,(long)PWM_MAX_RUN);
  pwM=constrain(pwM,PWM_MIN_RUN,PWM_MAX_RUN);
  // FIX#P9 (REVISI FIX#P13): floor PWM_MIN_STEADY dipindah ke pidStep() dengan guard e_raw>0
  // setPWM TIDAK lagi apply floor sendiri — biar pidStep yang kontrol penuh
  pidLastPWM=(uint8_t)pwM;
  analogWrite(PIN_PUMP,pidLastPWM);
}

void setPWMManual(int pw){
  if(pw<1){ analogWrite(PIN_PUMP,0); pidLastPWM=0; F.kickActive=0; return; }
  F.kickActive=0;
  int pwM=(int)map((long)pw,1L,255L,(long)PWM_MIN_RUN,(long)PWM_MAX_RUN);
  pwM=constrain(pwM,PWM_MIN_RUN,PWM_MAX_RUN);
  pidLastPWM=(uint8_t)pwM;
  analogWrite(PIN_PUMP,pidLastPWM);
}

void pidStep(float pv,float dt){
  float e_raw=pidSP-pv;
  bool inDeadband = (fabs(e_raw) < 1.0f);  // FIX#P7: perkecil deadband 2.5→1.0% (deadband lebar bikin PV settle sembarang dalam ±2.5%)
  if(inDeadband && !F.pidSettledOnce) F.pidSettledOnce=1;
  // FIX#P12: kalau ada integrator dari sesi sebelumnya (misal observer reconnect),
  // anggap sudah settled → pakai floor PWM_MIN_STEADY bukan langsung PWM_MIN_RUN
  if(!F.pidSettledOnce && pidInteg > 5.0f) F.pidSettledOnce=1;
  float e = inDeadband ? 0.0f : e_raw;
  e=cf(e,-25,25);
  float ff=pidKff*(pidSP-prevSP); prevSP=pidSP;
  if(ff<0) ff=0;
  float dRaw=0;
  if(dt>0.001f)dRaw=-pidKp*pidKd*(pv-pidPrevPV)/dt;
  dFiltered+=D_FILTER_A*(dRaw-dFiltered);
  dFiltered=cf(dFiltered,-10.0f,10.0f);
  float P=pidKp*e;
  float uUnsat=P+pidInteg+dFiltered+ff;
  float uSat=cf(uUnsat,0,100);
  float wErr=uSat-uUnsat;
  if(!inDeadband){
    pidInteg+=(pidKi*e*dt)+(ANTIWINDUP_KC*wErr*dt);
    pidInteg=cf(pidInteg,-100,100);
  } else {
    // FIX#P8+P10: deadband integrator — partial accumulation + antiwindup + decay arah atas
    float e_db = cf(e_raw, -1.0f, 1.0f);  // clamp ke ±deadband
    if (e_raw > 0.0f) {
      // PV di bawah SP: partial integrator 10% supaya bisa kompensasi outflow
      pidInteg += (pidKi * 0.1f * e_db * dt);
    } else {
      // FIX#P10: PV sudah di atas SP (e_raw <= 0) → DECAY integrator pelan
      // Ini rem integrator supaya tidak terus akumulasi ke atas
      pidInteg *= 0.999f;
    }
    // FIX#P10: antiwindup tetap aktif di dalam deadband
    // Kalau output saturasi, wErr akan negatif → tarik integrator turun
    pidInteg += (ANTIWINDUP_KC * wErr * dt);
    pidInteg = cf(pidInteg,-100,100);
  }
  float oRaw=cf(P+pidInteg+dFiltered+ff,0,100);
  float rr=(pv-pvPrev)/dt;
  pvRate+=0.3f*(rr-pvRate);
  pvPrev=pv;
  int o=(int)oRaw;
  if(pidLastPWM>60&&pvRate<-0.03f&&fabs(pidSP-pv)>5.0f)
    o=ci(o+(int)roundf(-pvRate*20.0f),0,100);
  // FIX#P9 + FIX#P13: floor PWM hanya aktif saat PV < SP (e_raw > 0)
  // Kalau PV sudah di atas SP (e_raw <= 0) → pompa HARUS bisa mati total
  // Ini fix utama: pompa tidak boleh jalan saat PV sudah lewat SP
  if(F.pidSettledOnce && e_raw > 0.0f && fabs(e_raw) < 5.0f && o < 1){
    o = 1;  // paksa minimal 1% → setPWM map ke PWM_MIN_RUN (pompa jalan pelan kompensasi outflow)
  }
  // FIX#P13: apply PWM_MIN_STEADY di sini, bukan di setPWM, supaya ada konteks e_raw
  // Kalau e_raw <= 0 (PV >= SP), o tetap 0 → setPWM(0) → pompa mati
  if(o > 0 && F.pidSettledOnce && e_raw > 0.0f){
    // Hitung pwm yang akan dihasilkan, kalau kurang dari STEADY, naikkan
    int pwmCalc = (int)map((long)o, 0, 100, (long)PWM_MIN_RUN, (long)PWM_MAX_RUN);
    if(pwmCalc < PWM_MIN_STEADY) o = 1;  // biarkan setPWM pakai PWM_MIN_RUN (lebih aman)
  }
  setPWM(o);
  pidPrevPV=pv;
}

// =============================================
//  AUTOTUNE v9.7 — FAST (FIX#60)
// =============================================
void cancelTune(uint8_t reasonIdx){
  F.tunActive=0; F.tunCancel=1; F.tunDone=0;
  F.tunReasonIdx=reasonIdx;
  F.tunFilling=0; F.tunDraining=0; F.tunDirectRelay=0;
  F.tunBiasEst=0; tunBiasEstPhase=0;  // FIX#AB1: reset bias estimation state
  pidInteg=0; tunSign=0; dFiltered=0;
  pidPrevPV=lastPV;
  // FIX#63: reset pvPrev & pvRate agar tidak spike saat PID resume
  pvPrev=lastPV; pvRate=0;
}

void startTune(){
  F.tunCancel=0; F.tunDone=0;
  F.tunReasonIdx=0;
  F.tunActive=1; F.pidOn=1;
  tunDrainStartMs=0;
  tunXcount=0; tunPVhi=-1e9f; tunPVlo=1e9f;
  tunPrevMs=0; tunLastMs=0; tunSign=0;
  pidInteg=0; dFiltered=0;
  pidLastPWM=0; F.kickActive=0;
  pidPrevPV=lastPV;
  // FIX#63: reset pvPrev & pvRate agar tidak stale saat pidStep pertama jalan
  pvPrev=lastPV; pvRate=0;

  // FIX#AB1: mulai dengan auto-bias estimation (10 detik) sebelum fill/drain/relay
  // Estimasi outflow rate dengan ukur dPV saat pompa ON 5 detik dan OFF 5 detik
  F.tunBiasEst=1; F.tunFilling=0; F.tunDraining=0; F.tunDirectRelay=0;
  tunBiasEstPhase=1;           // fase 1: pompa ON, ukur rate naik
  tunBiasEstPVstart=lastPV;
  tunBiasEstMs=millis();
  tunBiasEstUp=0; tunBiasEstDown=0;
  tunBiasDynamic=TUN_BIAS;     // default dulu, akan di-update setelah estimasi
}

void tuneStep(float pv){
  uint32_t now=millis();

  // --- FIX#AB1: FASE AUTO-BIAS ESTIMATION ---
  // Fase 1 (5 detik): pompa 100%, ukur rate naik
  // Fase 2 (5 detik): pompa 0%, ukur rate turun
  // Setelah selesai: hitung bias optimal, lanjut ke fill/drain/relay normal
  if(F.tunBiasEst){
    float elapsed = (float)(now - tunBiasEstMs) / 1000.0f;
    if(tunBiasEstPhase==1){
      setPWM(100);
      if(elapsed >= 5.0f){
        // Selesai fase 1: hitung rate naik
        float dPV = pv - tunBiasEstPVstart;
        tunBiasEstUp = (dPV > 0) ? (dPV / elapsed) : 0.01f;  // %/detik, min 0.01 biar tidak div/0
        // Mulai fase 2: pompa OFF
        tunBiasEstPhase=2;
        tunBiasEstPVstart=pv;
        tunBiasEstMs=now;
      }
    } else if(tunBiasEstPhase==2){
      stopPump();
      if(elapsed >= 5.0f){
        // Selesai fase 2: hitung rate turun
        float dPV = tunBiasEstPVstart - pv;   // positif kalau turun
        tunBiasEstDown = (dPV > 0) ? (dPV / elapsed) : 0.01f;
        // Hitung bias optimal: outflow_rate/(inflow_rate+outflow_rate)*100 + margin 20%
        float total = tunBiasEstUp + tunBiasEstDown;
        float biasCalc = (tunBiasEstDown / total) * 100.0f + 20.0f;
        tunBiasDynamic = cf(biasCalc, 30.0f, 70.0f);  // clamp 30-70%
        // Selesai estimasi — lanjut ke fill/drain/relay normal
        F.tunBiasEst=0;
        tunBiasEstPhase=3;  // done
        float pvDist = fabs(pv - pidSP);
        if(pvDist <= TUN_DIRECT_RELAY_THR){
          F.tunDirectRelay=1;
          tunSign = (pv < pidSP) ? 1.0f : -1.0f;
          tunFillStartMs=now; tunLastMs=now;
        } else if(pv < pidSP){
          F.tunFilling=1;
          tunFillStartMs=now;
        } else {
          F.tunDraining=1;
          tunDrainStartMs=now;
        }
      }
    }
    return;
  }

  // --- FASE FILL (kalau PV terlalu jauh di bawah SP) ---
  if(F.tunFilling){
    setPWM(100);
    if(now - tunFillStartMs > TUN_FILL_TIMEOUT_MS){
      // FIX#60: FILL timeout → tidak cancel, langsung masuk relay dari posisi sekarang
      F.tunFilling=0; F.tunDirectRelay=1;
      tunSign = (pv < pidSP) ? 1.0f : -1.0f;
      tunLastMs=now; tunFillStartMs=now;
      return;
    }
    if(pv >= pidSP - RELAY_BAND - 2.0f){
      F.tunFilling=0; F.tunDirectRelay=1;
      tunSign = (pv < pidSP) ? 1.0f : -1.0f;
      tunLastMs=now; tunFillStartMs=now;
    }
    return;
  }

  // --- FASE DRAIN (kalau PV terlalu jauh di atas SP) ---
  if(F.tunDraining){
    stopPump();
    if(now - tunDrainStartMs > TUN_DRAIN_TIMEOUT_MS){
      // FIX#60: DRAIN timeout → langsung masuk relay dari posisi sekarang
      F.tunDraining=0; F.tunDirectRelay=1;
      tunSign = (pv < pidSP) ? 1.0f : -1.0f;
      tunLastMs=now; tunFillStartMs=now;
      return;
    }
    if(pv <= pidSP + RELAY_BAND + 2.0f){
      F.tunDraining=0; F.tunDirectRelay=1;
      tunSign = (pv < pidSP) ? 1.0f : -1.0f;
      tunLastMs=now; tunFillStartMs=now;
    }
    return;
  }

  // --- FASE RELAY TEST (direct atau setelah fill/drain) ---
  // Update pvhi/pvlo (outlier rejection)
  {
    float mid=(tunXcount>0)?((tunPVhi+tunPVlo)/2.0f):pidSP;
    if(fabs(pv-mid)<2.0f*TUN_AMP || tunXcount==0){
      if(pv>tunPVhi) tunPVhi=pv;
      if(pv<tunPVlo) tunPVlo=pv;
    }
  }

  // Bang-bang dengan hysteresis band
  bool relayOn = (tunSign > 0);
  if(pv >= pidSP + RELAY_BAND) relayOn = false;
  if(pv <= pidSP - RELAY_BAND) relayOn = true;
  float newSign = relayOn ? 1.0f : -1.0f;

  // FIX#AB1: pakai tunBiasDynamic (hasil estimasi) bukan hardcoded TUN_BIAS
  if(relayOn) setPWM((int)cf(tunBiasDynamic+TUN_AMP,0,100));
  else        stopPump();

  if(tunPrevMs==0){ tunPrevMs=now; tunSign=newSign; return; }
  if(tunLastMs==0) tunLastMs=now;

  // Timeout check
  {
    uint32_t tmoMs=(tunXcount==0)?TUN_FIRST_CROSS_TIMEOUT_MS:TUN_HALF_CYCLE_TIMEOUT_MS;
    if(now - tunLastMs > tmoMs){ cancelTune(2); return; }
  }

  // Deteksi crossing
  if(newSign != tunSign){
    tunXcount++;
    tunSign=newSign;
    tunLastMs=now;
    if(tunXcount==1){ tunPVhi=-1e9f; tunPVlo=1e9f; tunPrevMs=now; }
    if(tunXcount>=tunNeed){
      float a=(tunPVhi-tunPVlo)/2.0f;
      if(a<0.1f){ cancelTune(0); return; }
      float Pu_raw=2.0f*(float)(now-tunPrevMs)/1000.0f/(float)(tunXcount-1);
      float Pu = Pu_raw * 0.75f;
      float relayD=TUN_AMP;  // FIX#P4: relayD harus = amplitudo relay (TUN_AMP), bukan (BIAS+AMP)*0.5
      float Ku=(4.0f*relayD)/(3.14159f*a);
      float newKp,newKi,newKd=0;
      // FIX#P6: Tyreus-Luyben formula + de-tune 0.6x
      // TL lebih konservatif dari ZN Classic → cocok untuk tangki asimetris (turun lambat)
      // De-tune 0.6x tambahan → pastikan no overshoot meski outflow sedang
      if(tmode==TM_PI){
        newKp = 0.3158f*Ku * 0.6f;           // TL PI: 0.3158*Ku, de-tune 0.6x
        float Ti = 2.2f*Pu;
        newKi = newKp / Ti;                   // Ki = Kp / Ti
      } else {
        newKp = 0.4545f*Ku * 0.6f;           // TL PID: 0.4545*Ku, de-tune 0.6x
        float Ti = 2.2f*Pu;
        float Td = Pu / 6.3f;
        newKi = newKp / Ti;                   // Ki = Kp / Ti
        newKd = newKp * Td * 0.6f;           // Kd de-tune 0.6x juga
      }
      float kdMax = cf(Pu * 0.125f, 0, 50.0f);
      pidKp=cf(newKp,0,500); pidKi=cf(newKi,0,500); pidKd=cf(newKd,0,kdMax);
      pidInteg=0; dFiltered=0;
      pidPrevPV=lastPV;
      prevSP=pidSP;
      F.tunActive=0; F.tunDone=1; F.tunCancel=0;
      F.tunFilling=0; F.tunDraining=0; F.tunDirectRelay=0;
      tunSign=0;
      eeMarkDirty();eeForceFlush();  // FIX#94: save PID tuning result LANGSUNG
    }
  }
}

// =============================================
//  READ PV
// =============================================
float readPV(){
  float d=readDist();
  if(isnan(d)){
    sensorErrCount++;
    if(sensorErrCount>SENSOR_ERR_MAX)sensorErrCount=SENSOR_ERR_MAX;
    return lastPV;
  }
  sensorErrCount=0;
  float sp=sLRV-sURV; if(sp<0.01f)sp=0.01f;
  float p=cf((sLRV-d)/sp*100.0f,0,100);
  float raw=p;
  mbuf[midx++]=raw;
  if(midx>=MED_N){F.mfill=1; midx=0;}
  float med;
  if(F.mfill){
    float s[MED_N]; memcpy(s,mbuf,sizeof(mbuf));
    for(int i=0;i<MED_N-1;i++)for(int j=i+1;j<MED_N;j++)if(s[j]<s[i]){float t=s[i];s[i]=s[j];s[j]=t;}
    med=s[MED_N/2];
  } else med=raw;
  if(!emaInitDone){emaVal=med;emaInitDone=true;}
  else emaVal+=EMA_A*(med-emaVal);
  pvS+=0.1f*(emaVal-pvS);
  // FIX#P11: validasi range — reject nilai gila akibat serial corruption
  if(emaVal < -5.0f || emaVal > 110.0f){
    sensorErrCount++;
    return lastPV;  // buang, pakai lastPV yang valid
  }
  return emaVal;
}

// =============================================
//  ESP COMM
// =============================================
void sendToESP(){
  int tunePhase=0;
  if(F.tunActive){
    if(F.tunBiasEst)        tunePhase=6;  // FIX#AB1: fase bias estimation
    else if(F.tunFilling)   tunePhase=1;
    else if(F.tunDraining)  tunePhase=2;
    else                    tunePhase=3;  // relay (termasuk direct relay)
  } else if(F.tunDone)      tunePhase=4;
  else if(F.tunCancel)      tunePhase=5;
  snprintf_P(shBuf,sizeof(shBuf),  // FIX#RAM2: pakai PSTR → format string di flash bukan RAM
    PSTR("DATA:pv=%d,sp=%d,out=%d,mode=%c,run=%d,ah=%d,al=%d,tune=%d"),
    (int)(lastPV+0.5f),(int)(pidSP+0.5f),
    pwmToDisplay(pidLastPWM),
    amode==AM_PID?'P':'M',
    F.pidOn?1:0,
    (F.alarmHiAct&&F.alarmHiEn)?1:0,
    (F.alarmLoAct&&F.alarmLoEn)?1:0,
    tunePhase
  );
  Serial.println(shBuf);
  snprintf_P(shBuf,sizeof(shBuf),  // FIX#RAM2
    PSTR("PIDK:kp=%d,ki=%d,kd=%d,en=%d%d"),
    (int)(pidKp*100),(int)(pidKi*1000),(int)(pidKd*1000),
    F.alarmHiEn?1:0,F.alarmLoEn?1:0
  );
  Serial.println(shBuf);
}

void readFromESP(){
  static uint32_t rxLastMs = 0;
  if(rxIdx > 0 && (millis() - rxLastMs) > 500UL) rxIdx = 0;
  while(Serial.available()){
    rxLastMs = millis();
    char c=(char)Serial.read();
    if(c=='\n'||rxIdx>=RXBUF_SIZE-1){
      rxBuf[rxIdx]=0; rxIdx=0;
      if(strncmp(rxBuf,"CMD:",4)==0){
        char* p=rxBuf+4;
        char* ff;
        ff=strstr(p,"sp=");
        if(ff){
          const char* spStr=ff+3;
          bool spValid=false;
          for(const char* sc=spStr;*sc&&*sc!=',';sc++){if(*sc>='0'&&*sc<='9'){spValid=true;break;}}
          float v=spValid?fromUnit(atof(spStr)):pidSP;
          if(spValid&&v>=0&&v<=100){
            float spDelta=fabs(v-pidSP);
            if(spDelta>=INTEG_SP_RESET_THR){ pidInteg=0; dFiltered=0; }
            pidSP=v; prevSP=v; eeMarkDirty();eeForceFlush();  // FIX#95: save SP dari ESP langsung
          }
        }
        ff=strstr(p,"mode=");
        if(ff){char m=*(ff+5);
          if(m=='P'){amode=AM_PID;F.pidOn=1;pidLastPWM=0;F.kickActive=0;pidInteg=0;dFiltered=0;pvPrev=lastPV;pvRate=0;}
          else if(m=='M'){amode=AM_MANUAL;F.pidOn=0;F.kickActive=0;}
        }
        ff=strstr(p,"pwm=");
        if(ff){int v=atoi(ff+4);if(v>=0&&v<=255)manPWM=(uint8_t)v;}
        ff=strstr(p,"run=");
        if(ff){int r=atoi(ff+4);
          if(r==1){F.pidOn=1;pidLastPWM=0;F.kickActive=0;pvPrev=lastPV;pvRate=0;}
          else if(r==0){F.pidOn=0;stopPump();}
        }
        ff=strstr(p,"kp=");
        if(ff){float v=atof(ff+3);if(v>=0&&v<=500){pidKp=v;pidInteg=0;dFiltered=0;eeMarkDirty();eeForceFlush();}}  // FIX#96
        ff=strstr(p,"ki=");
        if(ff){float v=atof(ff+3);if(v>=0&&v<=500){pidKi=v;pidInteg=0;eeMarkDirty();eeForceFlush();}}  // FIX#97
        ff=strstr(p,"kd=");
        if(ff){float v=atof(ff+3);if(v>=0&&v<=500){pidKd=v;dFiltered=0;eeMarkDirty();eeForceFlush();}}  // FIX#98
        ff=strstr(p,"ahen=");
        if(ff){F.alarmHiEn=(atoi(ff+5)!=0);eeMarkDirty();eeForceFlush();}  // FIX#90: immediate save
        ff=strstr(p,"alen=");
        if(ff){F.alarmLoEn=(atoi(ff+5)!=0);eeMarkDirty();eeForceFlush();}  // FIX#90: immediate save
      }
    } else if(c!='\r') rxBuf[rxIdx++]=c;
  }
}

// =============================================
//  MENU ITEMS
// =============================================
const char mi0[] PROGMEM="SET SP";
const char mi1[] PROGMEM="MODE";
const char mi2[] PROGMEM="SENSOR CALIB";
const char mi3[] PROGMEM="UNITS";
const char mi4[] PROGMEM="ALARM";
const char mi5[] PROGMEM="IP INFO";
const char* const mItems[] PROGMEM={mi0,mi1,mi2,mi3,mi4,mi5};
#define MENU_COUNT 6

void startEdit(ET t,Scr b){etgt=t;nbuf[0]=0;eback=b;scr=S_EDIT;}
void startEditSP(Scr b){
  etgt=E_SP; eback=b; scr=S_EDIT;
  char tmp[9];
  float spInUnit=toUnit(pidSP);
  if(unitMode==U_M) dtostrf(spInUnit,5,2,tmp);
  else snprintf_P(tmp,9,PSTR("%d"),(int)(spInUnit+0.5f));
  uint8_t s=0; while(tmp[s]==' ')s++;
  strncpy(nbuf,tmp+s,8); nbuf[8]=0;
}

void commitEdit(){
  if(!nbuf[0])return;
  {bool hasDigit=false;for(uint8_t _i=0;nbuf[_i];_i++){if(nbuf[_i]>='0'&&nbuf[_i]<='9'){hasDigit=true;break;}}if(!hasDigit){nbuf[0]=0;return;}}
  float fv=atof(nbuf);
  int iv=atoi(nbuf);
  switch(etgt){
    case E_SP: {
      float newSP=fromUnit(fv);
      float spDelta=fabs(newSP-pidSP);
      if(spDelta>=INTEG_SP_RESET_THR){ pidInteg=0; dFiltered=0; }
      pidSP=newSP; prevSP=pidSP;
      break;
    }
    case E_KP:   pidKp=cf(fv,0,500);pidInteg=0;dFiltered=0;break;
    case E_KI:   pidKi=cf(fv,0,500);pidInteg=0;break;
    case E_KD:   pidKd=cf(fv,0,500);dFiltered=0;break;
    case E_LRV:  sLRV=cf(fv,0,400); resetSensorFilter(); break;  // FIX#81
    case E_URV:  sURV=cf(fv,0,400); resetSensorFilter(); break;  // FIX#81
    case E_MPWM: manPWM=(uint8_t)constrain(iv,0,255);break;
    case E_ALHI: alarmHi=cf(fv,0,100);break;
    case E_ALLO: alarmLo=cf(fv,0,100);break;
    case E_FF:   pidKff=cf(fv,0,10);break;
    default:break;
  }
  etgt=E_NO; nbuf[0]=0; eeMarkDirty(); eeForceFlush();  // FIX#83: langsung flush, tidak tunggu 5 detik
}

void goBack(){
  if(scr==S_EDIT){scr=eback;return;}
  if(scr==S_MENU){scr=S_STAT;cur=0;mscr=0;return;}
  if(scr==S_MODE||scr==S_CAL||scr==S_UNIT||scr==S_ALARM||scr==S_IP){scr=S_MENU;cur=0;mscr=0;return;}
  if(scr==S_PSUB){scr=S_MODE;cur=0;return;}
  if(scr==S_ARUN){if(F.tunActive)cancelTune(1);scr=S_PSUB;cur=(tmode==TM_PI)?0:1;return;}
  if(scr==S_MTUN){scr=S_PSUB;cur=2;return;}
  if(scr==S_MPWM){scr=S_STAT;return;}
  if(scr==S_CALV||scr==S_CALU||scr==S_CALD){
    if(scr==S_CALU && F.urvSetDone){eeMarkDirty();eeForceFlush();}  // FIX#89(was#74): force save URV saat back
    if(scr==S_CALV){eeMarkDirty();eeForceFlush();}  // FIX#92: save sLRV saat back dari S_CALV
    if(scr==S_CALD){eeMarkDirty();eeForceFlush();}  // FIX#93: pastikan tersimpan saat keluar S_CALD
    scr=S_CAL;return;
  }
  scr=S_STAT;cur=0;mscr=0;
}

// =============================================
//  DRAW UI
// =============================================
void dStat(){
  snprintf_P(lb,21,PSTR("OUT:%-3d%% %s %s "),
    pwmToDisplay(pidLastPWM),
    amode==AM_MANUAL?"[M]":"[P]",
    (F.alarmHiAct&&F.alarmHiEn)?"!HI":(F.alarmLoAct&&F.alarmLoEn)?"!LO":"   "
  );
  LP(0,lb);
  LP_P(1,PSTR("--------------------"));
  char vb[5];
  fmtVal(toUnit(lastPV),vb);snprintf_P(lb,21,PSTR("PV :%s %s"),vb,getUL());LP(2,lb);
  fmtVal(toUnit(pidSP),vb);snprintf_P(lb,21,PSTR("SP :%s %s"),vb,getUL());LP(3,lb);
}
void dMenu(){
  LP_P(0,PSTR("=== MENU ===        "));
  char ib[16];
  for(uint8_t r=0;r<3;r++){
    uint8_t i=mscr+r;
    if(i>=MENU_COUNT){LP_P(r+1,PSTR(""));continue;}
    strcpy_P(ib,(const char*)pgm_read_ptr(&mItems[i]));
    snprintf_P(lb,21,PSTR("%s%-19s"),i==cur?">":" ",ib);LP(r+1,lb);
  }
}
void dMode(){
  LP_P(0,PSTR("MODE:               "));
  LP_P(1,cur==0?PSTR(">PID               "):PSTR(" PID               "));
  LP_P(2,cur==1?PSTR(">MANUAL            "):PSTR(" MANUAL            "));
  LP_P(3,PSTR("A=Back  B=Select    "));
}
void dPSub(){
  LP_P(0,PSTR("PID MODE:           "));
  LP_P(1,cur==0?PSTR(">AUTO TUNE PI      "):PSTR(" AUTO TUNE PI      "));
  LP_P(2,cur==1?PSTR(">AUTO TUNE PID     "):PSTR(" AUTO TUNE PID     "));
  LP_P(3,cur==2?PSTR(">MANUAL TUNE       "):PSTR(" MANUAL TUNE       "));
}

void dArun(){
  const char* m=(tmode==TM_PI)?"PI":"PID";
  if(F.tunActive){
    if(F.tunBiasEst){
      // FIX#AB1: tampil fase estimasi bias di LCD
      snprintf_P(lb,21,PSTR("AUTO %s EST BIAS"),m); LP(0,lb);
      snprintf_P(lb,21,PSTR("PH%d up=%-3d dn=%-3d  "),tunBiasEstPhase,(int)(tunBiasEstUp*100),(int)(tunBiasEstDown*100)); LP(1,lb);
      dtostrf(lastPV,4,1,ta); snprintf_P(lb,21,PSTR("PV:%s%% B=%d%%       "),ta,(int)tunBiasDynamic); LP(2,lb);
      LP_P(3,PSTR("estimating...       "));
    } else if(F.tunFilling){
      snprintf_P(lb,21,PSTR("AUTO %s FILLING..."),m); LP(0,lb);
      dtostrf(lastPV,4,1,ta); dtostrf(pidSP,4,1,tb);
      snprintf_P(lb,21,PSTR("PV:%s%% SP:%s%%  "),ta,tb); LP(1,lb);
      uint32_t elapsed=(millis()-tunFillStartMs)/1000UL;
      uint32_t rem=(TUN_FILL_TIMEOUT_MS/1000UL>elapsed)?(TUN_FILL_TIMEOUT_MS/1000UL-elapsed):0UL;
      snprintf_P(lb,21,PSTR("Tmo:%4lus left      "),rem); LP(2,lb);
      LP_P(3,PSTR("A=Back  C=Cancel    "));
    } else if(F.tunDraining){
      snprintf_P(lb,21,PSTR("AUTO %s DRAINING "),m); LP(0,lb);
      dtostrf(lastPV,4,1,ta); dtostrf(pidSP,4,1,tb);
      snprintf_P(lb,21,PSTR("PV:%s%% SP:%s%%  "),ta,tb); LP(1,lb);
      uint32_t dElapsed=(millis()-tunDrainStartMs)/1000UL;
      uint32_t dRem=(TUN_DRAIN_TIMEOUT_MS/1000UL>dElapsed)?(TUN_DRAIN_TIMEOUT_MS/1000UL-dElapsed):0UL;
      snprintf_P(lb,21,PSTR("Tmo:%4lus left      "),dRem); LP(2,lb);
      LP_P(3,PSTR("A=Back  C=Cancel    "));
    } else {
      // Relay phase (termasuk direct relay FIX#60)
      snprintf_P(lb,21,PSTR("AUTO %s RELAY    "),m); LP(0,lb);
      dtostrf(lastPV,4,1,ta); dtostrf(pidSP,4,1,tb);
      snprintf_P(lb,21,PSTR("PV:%s SP:%s%%   "),ta,tb); LP(1,lb);
      {
        uint32_t tmoMs=(tunXcount==0)?TUN_FIRST_CROSS_TIMEOUT_MS:TUN_HALF_CYCLE_TIMEOUT_MS;
        uint32_t relElapsed=(tunLastMs>0)?(millis()-tunLastMs)/1000UL:0UL;
        uint32_t relRem=(tmoMs/1000UL>relElapsed)?(tmoMs/1000UL-relElapsed):0UL;
        snprintf_P(lb,21,PSTR("O:%3d%% x:%2d t:%3lus"),pwmToDisplay(pidLastPWM),tunXcount,relRem); LP(2,lb);
      }
      LP_P(3,PSTR("A=Back  C=Cancel    "));
    }
  } else if(F.tunCancel){
    snprintf_P(lb,21,PSTR("AUTO %s CANCEL  "),m); LP(0,lb);
    char rb[9]; strcpy_P(rb,(const char*)pgm_read_ptr(&trT[F.tunReasonIdx]));
    LP(1,rb); LP_P(2,PSTR("")); LP_P(3,PSTR("A=Back              "));
  } else {
    snprintf_P(lb,21,PSTR("AUTO %s DONE    "),m); LP(0,lb);
    dtostrf(pidKp,6,2,ta); snprintf_P(lb,21,PSTR("KP:%s "),ta); LP(1,lb);
    dtostrf(pidKi,6,2,ta); dtostrf(pidKd,6,2,tb);
    snprintf_P(lb,21,PSTR("KI:%s KD:%s "),ta,tb); LP(2,lb);
    LP_P(3,PSTR("A=Back              "));
  }
}

void dMtun(){
  LP_P(0,PSTR("MANUAL TUNE B=EDIT  "));
  dtostrf(pidKp,6,2,ta);snprintf_P(lb,21,PSTR("%sKP %s  "),cur==0?">":" ",ta);LP(1,lb);
  dtostrf(pidKi,6,2,ta);snprintf_P(lb,21,PSTR("%sKI %s  "),cur==1?">":" ",ta);LP(2,lb);
  dtostrf(pidKd,6,2,ta);snprintf_P(lb,21,PSTR("%sKD %s  "),cur==2?">":" ",ta);LP(3,lb);
}

void dMpwm(){
  char pv[5],sp[5];
  fmtVal(toUnit(lastPV),pv);fmtVal(toUnit(pidSP),sp);
  LP_P(0,PSTR("MANUAL PWM CONTROL  "));
  snprintf_P(lb,21,PSTR("PV:%s SP:%s %s  "),pv,sp,getUL());LP(1,lb);
  int manPct = (manPWM==0)?0:pwmToDisplay(
    (uint8_t)constrain(map((long)manPWM,1L,255L,(long)PWM_MIN_RUN,(long)PWM_MAX_RUN),
    PWM_MIN_RUN,PWM_MAX_RUN));
  snprintf_P(lb,21,PSTR("PWM:%3d%% (manual)   "),manPct);LP(2,lb);
  LP_P(3,PSTR("B=EDIT 2/8=+/- C=Bk "));
}

void dAlarm(){
  LP_P(0,PSTR("ALARM  B=EDIT D=TOG "));
  snprintf_P(lb,21,PSTR("%sHI %3d%% [%s]  "),cur==0?">":" ",(int)alarmHi,F.alarmHiEn?"ON ":"OFF");LP(1,lb);
  snprintf_P(lb,21,PSTR("%sLO %3d%% [%s]  "),cur==1?">":" ",(int)alarmLo,F.alarmLoEn?"ON ":"OFF");LP(2,lb);
  LP_P(3,PSTR("A=Back  2/8=Select  "));
}
void dIP(){
  LP_P(0,PSTR("ESP8266 - IoT CTRL  "));
  LP_P(1,PSTR("NodeMCU 1.0 ESP-12E "));
  LP_P(2,PSTR("HW Serial pin 0(RX) "));
  LP_P(3,PSTR("pin 1(TX) ke ESP    "));
}
void dCal(){
  dtostrf(sRAW,5,1,ta);
  LP_P(0,PSTR("SENSOR CALIB        "));
  snprintf_P(lb,21,PSTR("RAW:%s cm       "),ta);LP(1,lb);
  LP_P(2,cur==0?PSTR(">SET LRV (0%)   "):PSTR(" SET LRV (0%)   "));
  LP_P(3,cur==1?PSTR(">SET URV (100%) "):PSTR(" SET URV (100%) "));
}
void dCalSub(bool isL){
  dtostrf(sRAW,5,1,ta);dtostrf(isL?sLRV:sURV,5,1,tb);
  LP_P(0,isL?PSTR("SET LRV (0%)     "):PSTR("SET URV (100%)  "));
  snprintf_P(lb,21,PSTR("RAW:%s cm       "),ta);LP(1,lb);
  snprintf_P(lb,21,PSTR("%s:%s cm       "),isL?"LRV":"URV",tb);LP(2,lb);
  if(!isL){
    // FIX#73: warning kalau LRV<=URV setelah B dipencet
    if(sLRV <= sURV+1.0f && F.urvSetDone){
      LP_P(3,PSTR("!LRV<=URV B=ReSet "));
    } else {
      LP_P(3,F.urvSetDone ? PSTR("URV SET! #=Done A=Bk") : PSTR("ISI PENUH B=URV    "));  // FIX#88
    }
  } else {
    LP_P(3,PSTR("B=LRV #=Next A=Bk  "));  // FIX#87: instruksi lebih jelas
  }
}
void dCalD(){
  dtostrf(sLRV,5,1,ta);dtostrf(sURV,5,1,tb);
  LP_P(0,PSTR("CALIBRATION OK!     "));
  snprintf_P(lb,21,PSTR("LRV: %s cm      "),ta);LP(1,lb);
  snprintf_P(lb,21,PSTR("URV: %s cm      "),tb);LP(2,lb);
  LP_P(3,PSTR("A=Back to Menu      "));
}
void dUnit(){
  LP_P(0,PSTR("SELECT UNIT:        "));
  char ub[5];
  for(uint8_t r=0;r<3;r++){
    uint8_t i=unitScr+r;
    if(i>=4){LP_P(r+1,PSTR(""));continue;}
    strcpy_P(ub,(const char*)pgm_read_ptr(&ulT[i]));
    snprintf_P(lb,21,PSTR("%s%-18s"),i==cur?">":" ",ub);LP(r+1,lb);
  }
}
void dEdit(){
  LP_P(0,PSTR("EDIT VALUE          "));
  snprintf_P(lb,21,PSTR("Input: %-13s"),nbuf); LP(1,lb);
  if(etgt==E_SP){
    snprintf_P(lb,21,PSTR("Unit : %-13s"),getUL()); LP(2,lb);
  } else {
    LP_P(2,PSTR("D=decimal  *=del    "));
  }
  LP_P(3,PSTR("A=Cancel   #=Enter  "));
}

void drawUI(){
  static Scr ls=(Scr)255;
  if(scr!=ls){lcd.clear();ls=scr;}
  switch(scr){
    case S_SPLASH: LP_P(0,PSTR("  AquaFlow v10.5  ")); LP_P(1,PSTR("  PID Controller  ")); break;  // FIX#85-90
    case S_STAT:  dStat();break;
    case S_MENU:  dMenu();break;
    case S_MODE:  dMode();break;
    case S_PSUB:  dPSub();break;
    case S_ARUN:  dArun();break;
    case S_MTUN:  dMtun();break;
    case S_MPWM:  dMpwm();break;
    case S_CAL:   dCal();break;
    case S_CALV:  dCalSub(true);break;
    case S_CALU:  dCalSub(false);break;
    case S_CALD:  dCalD();break;
    case S_UNIT:  dUnit();break;
    case S_EDIT:  dEdit();break;
    case S_ALARM: dAlarm();break;
    case S_IP:    dIP();break;
    default:break;
  }
}

// =============================================
//  KEY HANDLER
// =============================================
void processPWMHold(uint32_t now){
  if(lastKeyPWM==0) return;
  bool held=false;
  for(uint8_t i=0;i<LIST_MAX;i++){
    if(kpd.key[i].kchar==lastKeyPWM &&
      (kpd.key[i].kstate==PRESSED||kpd.key[i].kstate==HOLD)){
      held=true; break;
    }
  }
  if(!held){ lastKeyPWM=0; return; }
  uint32_t elapsed=now-keyPressTime;
  if(elapsed<400) return;
  static uint32_t lastIncMs=0;
  if(now-lastIncMs<80) return;
  lastIncMs=now;
  if(lastKeyPWM=='8'){ int v=(int)manPWM+1; manPWM=(uint8_t)(v>255?255:v); }
  else if(lastKeyPWM=='2'){ int v=(int)manPWM-1; manPWM=(uint8_t)(v<0?0:v); }
}

void handleKey(char k){
  if(k=='D'&&scr==S_EDIT){
    uint8_t l=strlen(nbuf);
    if(!strchr(nbuf,'.')&&l<8){nbuf[l]='.';nbuf[l+1]=0;}
    return;
  }
  if(k=='D'&&scr==S_ALARM){
    if(cur==0){F.alarmHiEn=!F.alarmHiEn;eeMarkDirty();eeForceFlush();}  // FIX#77
    else      {F.alarmLoEn=!F.alarmLoEn;eeMarkDirty();eeForceFlush();}  // FIX#77
    return;
  }
  if(k=='A'){lastKeyPWM=0;goBack();return;}
  if(k=='C'){
    lastKeyPWM=0;
    if(F.tunActive){cancelTune(1);return;}
    if(scr==S_MPWM){scr=S_STAT;return;}
    return;
  }
  switch(scr){
    case S_STAT:
      if(k=='#'){scr=S_MENU;cur=0;mscr=0;}
      break;
    case S_MENU:
      if(k=='8'){cur=(cur+1)%MENU_COUNT;if(cur<mscr)mscr=cur;if(cur>=mscr+3)mscr=cur-2;}
      else if(k=='2'){cur=(cur+MENU_COUNT-1)%MENU_COUNT;if(cur<mscr)mscr=cur;if(cur>=mscr+3)mscr=cur-2;}
      else if(k=='B'||k=='#'){
        switch(cur){
          case 0: startEditSP(S_MENU); break;
          case 1:scr=S_MODE;cur=0;break;
          case 2:scr=S_CAL;cur=0;break;
          case 3:scr=S_UNIT;cur=(uint8_t)unitMode;break;
          case 4:scr=S_ALARM;cur=0;break;
          case 5:scr=S_IP;break;
        }
      }
      break;
    case S_MODE:
      if(k=='8'||k=='2')cur=(cur+1)%2;
      else if(k=='B'){
        if(cur==0){scr=S_PSUB;cur=0;amode=AM_PID;}
        else{amode=AM_MANUAL;manPWM=0;F.pidOn=0;F.kickActive=0;stopPump();scr=S_MPWM;cur=0;}
      }
      break;
    case S_PSUB:
      if(k=='8'||k=='2')cur=(cur+1)%3;
      else if(k=='B'){
        amode=AM_PID;pidLastPWM=0;F.kickActive=0;
        if(cur==0){tmode=TM_PI;startTune();scr=S_ARUN;}
        else if(cur==1){tmode=TM_PID;startTune();scr=S_ARUN;}
        else{F.pidOn=1;scr=S_MTUN;cur=0;}
      }
      break;
    case S_MTUN:
      if(k=='8'||k=='2')cur=(cur+1)%3;
      else if(k=='B'){
        if(cur==0)startEdit(E_KP,S_MTUN);
        else if(cur==1)startEdit(E_KI,S_MTUN);
        else startEdit(E_KD,S_MTUN);
      }
      break;
    case S_MPWM:
      if(k=='8'||k=='2'){
        if(lastKeyPWM!=k){ lastKeyPWM=k; keyPressTime=millis(); }
        if(k=='8'){ int v=(int)manPWM+1; manPWM=(uint8_t)(v>255?255:v); }
        else if(k=='2'){ int v=(int)manPWM-1; manPWM=(uint8_t)(v<0?0:v); }
      }
      else if(k=='B'){ lastKeyPWM=0; startEdit(E_MPWM,S_MPWM); }
      break;
    case S_CAL:
      if(k=='8'||k=='2')cur=(cur+1)%2;
      else if(k=='B'){
        if(cur==0){sLRV=sRAW;F.urvSetDone=0;scr=S_CALV;}  // FIX#85: reset urvSetDone saat mulai kalibrasi LRV baru
        else{sURV=sRAW;F.urvSetDone=1;scr=S_CALU;}  // FIX#80: set urvSetDone=1 langsung, URV sudah valid
      }
      break;
    case S_CALV:
      if(k=='B') sLRV=sRAW;
      else if(k=='#'){
        // FIX#86: JANGAN auto-set sURV=sRAW saat masuk S_CALU dari S_CALV
        // FIX#78 yang lama menyebabkan bug: sRAW di S_CALV = posisi tangki KOSONG (sama dengan sLRV)
        // → sURV=sLRV → span=0 → ERR:LRV<=URV! saat kalibrasi ke-2+
        // Fix yang benar: urvSetDone=0, user HARUS pindah ke posisi tangki penuh lalu pencet B
        F.urvSetDone=0;  // FIX#86: reset, user harus set URV manual di posisi tangki penuh
        scr=S_CALU;
      }
      break;
    case S_CALU:
      if(k=='B'){
        // User bisa update URV kalau sudah pindah posisi ke level penuh
        sURV=sRAW;
        F.urvSetDone=1;
      }
      else if(k=='#'){
        if(sLRV>sURV+1.0f){
          scr=S_CALD;
          resetSensorFilter();
          eeMarkDirty();
          eeForceFlush();  // FIX#77: simpan kalibrasi LANGSUNG
        } else {
          // FIX#73: tampilkan error jelas di LCD
          lcd.setCursor(0,3);
          lcd.print(F("ERR:LRV<=URV! B=Set "));  // FIX#RAM3: F() macro → string di flash
        }
      }
      break;
    case S_ALARM:
      if(k=='8'||k=='2')cur=(cur+1)%2;
      else if(k=='B'){
        if(cur==0)startEdit(E_ALHI,S_ALARM);
        else startEdit(E_ALLO,S_ALARM);
      }
      break;
    case S_UNIT:
      if(k=='8'){cur=(cur+1)%4;if(cur<unitScr)unitScr=cur;if(cur>=unitScr+3)unitScr=cur-2;}
      else if(k=='2'){cur=(uint8_t)((cur+3)%4);if(cur<unitScr)unitScr=cur;if(cur>=unitScr+3)unitScr=cur-2;}
      else if(k=='B'||k=='#'){unitMode=(UnitMode)cur;prevSP=pidSP;eeMarkDirty();eeForceFlush();scr=S_MENU;cur=0;mscr=0;}  // FIX#75+77: reset prevSP + force save unit
      break;
    case S_EDIT:{
      uint8_t l=strlen(nbuf);
      if(k=='*'){if(l>0)nbuf[l-1]=0;}
      else if(k=='#'){commitEdit();goBack();}
      else if(k>='0'&&k<='9'){if(l<8){nbuf[l]=k;nbuf[l+1]=0;}}
      break;
    }
    default:break;
  }
}

// =============================================
//  SETUP & LOOP
// =============================================
void setup(){
  delay(500);
  pinMode(PIN_TRIG,OUTPUT);
  pinMode(PIN_ECHO,INPUT);
  pinMode(PIN_PUMP,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO),echoISR,CHANGE);
  Serial.begin(115200);
  Wire.begin();
  delay(100);
  lcd.init(); lcd.init();
  lcd.backlight();
  delay(100);
  memset(mbuf,0,sizeof(mbuf));

  unitMode=U_PCT; pidSP=50; pidKp=4.0f; pidKi=0.05f; pidKd=0;
  pidKff=0.5f; sLRV=50; sURV=5; alarmHi=90; alarmLo=10;
  F.alarmHiEn=0; F.alarmLoEn=0;  // FIX#68: default alarm OFF — user harus aktifkan manual

  eeLoad();
  stopPump();
  amode=AM_PID; F.pidOn=1; pidInteg=0; dFiltered=0;
  F.pidSettledOnce=0;
  pidLastPWM=0; F.kickActive=0;
  F.urvSetDone=0;
  scr=S_STAT; cur=0; mscr=0;
  prevSP=pidSP;
  pvPrev=0;
  pvRate=0;
  tCtrl=millis(); tLCD=millis();
  eeLastMs = millis() - EE_MIN;
}

void loop(){
  readFromESP();  // FIX#70: hapus double call (was called twice, causing jitter)
  uint32_t now=millis();
  sonarUpdate();
  if(scr==S_MPWM) processPWMHold(now);
  char k=kpd.getKey();
  if(k){handleKey(k);tLCD=now-300UL;}
  if(now-tCtrl>=Ts_MS){
    float dt=cf((float)(now-tCtrl)/1000.0f,0.02f,DT_MAX);
    tCtrl=now;
    float pv=readPV();
    if(sensorErrCount>=SENSOR_ERR_MAX){
      stopPump();pidInteg=0;dFiltered=0;
    } else {
      lastPV=pv;
      F.alarmHiAct=(pv>alarmHi)?1:0;
      F.alarmLoAct=(pv<alarmLo)?1:0;
      if(F.alarmHiAct&&F.alarmHiEn){stopPump();pidInteg=0;dFiltered=0;}
      // FIX#65: alarmLo skip saat startup (pidSettledOnce=0) KECUALI saat autotune cancel
      else if(F.alarmLoAct&&F.alarmLoEn&&(F.pidSettledOnce||F.tunCancel)){stopPump();pidInteg=0;dFiltered=0;}
      else if(amode==AM_MANUAL){setPWMManual(manPWM);}
      else if(F.pidOn){
        if(F.tunActive)tuneStep(pv);
        else pidStep(pv,dt);
      } else {stopPump();}
      // FIX#64: pidPrevPV sudah di-handle dalam pidStep(), tidak perlu di-set lagi di sini
      // (dulu: pidPrevPV=pv; di sini — double assignment, removed)
    }
  }
  if(now-espLastMs>=ESP_INTERVAL){espLastMs=now;sendToESP();}
  eeFlush(now);
  if(now-tLCD>=300){tLCD=now;drawUI();}
}


