/*
 * ══════════════════════════════════════════════════════════════════
 *  AnaROV — Teknofest Video Görev Firmware'ı (Standalone)
 *  Jetson KULLANILMAZ — Tüm görev Deneyap üzerinde çalışır
 * ══════════════════════════════════════════════════════════════════
 *
 *  PARKUR (İleri Kategori — Dikdörtgen + Daire):
 *
 *        ┌──── 15s düz ─────── ⭕ (daire, min 1m çap)
 *        │                      │
 *    15s düz                  15s düz
 *        │                      │
 *    [BAŞLANGIÇ] ← 15s düz ────┘
 *        1m × 1m
 *
 *  DURUM AKIŞI:
 *    WAIT → DIVE → FWD1 → TURN1 → FWD2 → CIRCLE → TURN2 → FWD3 → TURN3 → FWD4 → SURFACE → DONE
 *
 *  MOTOR HARİTASI:
 *    M1 = Ön Sağ (Yatay)    M2 = Ön Sol (Yatay)
 *    M3 = Arka Sağ (Yatay)  M4 = Arka Sol (Yatay)
 *    M5 = Dikey Sol          M6 = Dikey Sağ (ters montaj)
 *
 *  MIXER:
 *    FR = fwd - strafe - yaw    FL = fwd + strafe + yaw
 *    RR = fwd + strafe - yaw    RL = fwd - strafe + yaw
 *    VF = dive                  VR = dive
 */

#include <ESP32Servo.h>
#include <mcp_can.h>
#include <SPI.h>
#include <Wire.h>
#include <Deneyap_6EksenAtaletselOlcumBirimi.h>
#include <PID_v1.h>
#include <math.h>

// ══════════════════════════════════════════════════════════════════
//  ⚙️  AYARLANACAK PARAMETRELER — BURADAN AYARLAYIN
//  Havuz testlerinde bu değerleri değiştirerek aracı kalibre edin.
// ══════════════════════════════════════════════════════════════════

// --- Güç Sınırı (%) ---
// 100 = tam güç. İlk testlerde 40-50 ile başlayın!
#define POWER_LIMIT  50

// --- Süre Ayarları (saniye) ---
// Her birini havuzunuza göre ayarlayın. Düz gidiş min 15s olmalı!
#define DURATION_WAIT        10.0   // Başlangıç bekleme (batarya takıp suya koymak için)
#define DURATION_DIVE         4.0   // Dalış süresi (yeterince batması için)
#define DURATION_FORWARD     16.0   // Her düz gitme süresi (min 15s — biraz fazla koyun)
#define DURATION_TURN_90      2.5   // 90° dönüş süresi (havuzda kalibre edin!)
#define DURATION_CIRCLE      12.0   // 360° daire süresi
#define DURATION_SURFACE      5.0   // Yüzeye çıkış süresi

// --- Hız Ayarları (0.0 - 1.0 arası) ---
// 1.0 = tam güç yüzdesi. Önerilen: 0.3 - 0.5 arası.
#define SPEED_FORWARD   0.40   // Düz gitme hızı
#define SPEED_TURN      0.50   // Dönüş hızı (yaw)
#define SPEED_DIVE      0.50   // Dalış hızı (agresif, sadece DIVE durumunda)
#define SPEED_HOLD      0.20   // Derinlik koruma (görev boyunca hafif aşağı itki)
#define SPEED_SURFACE   0.40   // Yüzeye çıkış hızı

// --- Daire Ayarları ---
#define CIRCLE_FWD      0.30   // Daire sırasında ileri hız
#define CIRCLE_YAW      0.45   // Daire sırasında dönüş hızı

// --- Roll PID Ayarları ---
#define ROLL_KP   1.5
#define ROLL_KI   0.0
#define ROLL_KD   0.25

// ══════════════════════════════════════════════════════════════════
//  DONANIM PİN TANIMLARI
// ══════════════════════════════════════════════════════════════════
#define SPI_CS 21      // D4 (CAN)
#define PIN_M1 2       // D9  (Ön Sağ)
#define PIN_M2 12      // D13 (Ön Sol)
#define PIN_M3 23      // D0  (Arka Sağ)
#define PIN_M4 14      // D14 (Arka Sol)
#define PIN_M5 13      // D12 (Dikey SOL)
#define PIN_M6 22      // D1  (Dikey SAĞ — ters montaj)
#define PIN_HALL_EFFECT 0  // D8 (Manyetik Şalter)

// ══════════════════════════════════════════════════════════════════
//  DURUM MAKİNESİ (STATE MACHINE)
// ══════════════════════════════════════════════════════════════════
enum MissionState {
  STATE_WAIT,             // 0: Başlangıç bekleme
  STATE_DIVE,             // 1: Su altına dalış
  STATE_FORWARD_1,        // 2: 1. düzlük (Kuzey)
  STATE_TURN_1,           // 3: Sağa 90° (→ Doğu)
  STATE_FORWARD_2,        // 4: 2. düzlük (Doğu)
  STATE_CIRCLE,           // 5: 360° daire (min 1m çap)
  STATE_TURN_AFTER_CIRCLE,// 6: Daire sonrası sağa 90° (→ Güney)
  STATE_FORWARD_3,        // 7: 3. düzlük (Güney)
  STATE_TURN_2,           // 8: Sağa 90° (→ Batı)
  STATE_FORWARD_4,        // 9: 4. düzlük (Batı → başlangıca dönüş)
  STATE_SURFACE,          // 10: Yüzeye çıkış
  STATE_FINISHED          // 11: Görev tamamlandı
};

MissionState currentState = STATE_WAIT;
unsigned long stateStartTime = 0;

// State isimleri (debug için)
const char* stateNames[] = {
  "WAIT", "DIVE", "FWD_1", "TURN_1", "FWD_2",
  "CIRCLE", "TURN_AFT", "FWD_3", "TURN_2", "FWD_4",
  "SURFACE", "FINISHED"
};

// ══════════════════════════════════════════════════════════════════
//  DONANIM NESNELERİ
// ══════════════════════════════════════════════════════════════════
MCP_CAN CAN(SPI_CS);
LSM6DSM IMU;
bool imuOK = false;

Servo esc_m1, esc_m2, esc_m3, esc_m4, esc_m5, esc_m6;

// PID
double roll_input, roll_output;
double roll_setpoint = 0.0;
double Kp = ROLL_KP, Ki = ROLL_KI, Kd = ROLL_KD;
PID rollPID(&roll_input, &roll_output, &roll_setpoint, Kp, Ki, Kd, DIRECT);

// Motor PWM değerleri
int pwm_m1 = 1500, pwm_m2 = 1500, pwm_m3 = 1500;
int pwm_m4 = 1500, pwm_m5 = 1500, pwm_m6 = 1500;

// Debug zamanlama
unsigned long lastPrintTime = 0;
const int PRINT_INTERVAL = 500;

// ══════════════════════════════════════════════════════════════════
//  THRUSTER MİXER
//  (forward, strafe, yaw, dive) → 6 motor PWM değeri
//  Giriş: -1.0 ... +1.0, Çıkış: 1000 ... 2000 µs
// ══════════════════════════════════════════════════════════════════
void thrusterMix(float fwd, float strafe, float yaw, float dive) {
  // Yatay motorlar (vektörel karıştırma)
  float m_fr = fwd - strafe - yaw;
  float m_fl = fwd + strafe + yaw;
  float m_rr = fwd + strafe - yaw;
  float m_rl = fwd - strafe + yaw;

  // Dikey motorlar
  float m_vf = dive;
  float m_vr = dive;

  // Sınırla, yüzdeye çevir, PWM'e map'le
  int pct_fr = constrain((int)(m_fr * POWER_LIMIT), -100, 100);
  int pct_fl = constrain((int)(m_fl * POWER_LIMIT), -100, 100);
  int pct_rr = constrain((int)(m_rr * POWER_LIMIT), -100, 100);
  int pct_rl = constrain((int)(m_rl * POWER_LIMIT), -100, 100);
  int pct_vf = constrain((int)(m_vf * POWER_LIMIT), -100, 100);
  int pct_vr = constrain((int)(m_vr * POWER_LIMIT), -100, 100);

  pwm_m1 = map(pct_fr, -100, 100, 1000, 2000);
  pwm_m2 = map(pct_fl, -100, 100, 1000, 2000);
  pwm_m3 = map(pct_rr, -100, 100, 1000, 2000);
  pwm_m4 = map(pct_rl, -100, 100, 1000, 2000);
  pwm_m5 = map(pct_vf, -100, 100, 1000, 2000);
  pwm_m6 = map(pct_vr, -100, 100, 2000, 1000); // M6 ters montaj!
}

// ══════════════════════════════════════════════════════════════════
//  DURUM DEĞİŞTİRME
// ══════════════════════════════════════════════════════════════════
void changeState(MissionState newState) {
  Serial.print("🔄 DURUM: ");
  Serial.print(stateNames[currentState]);
  Serial.print(" → ");
  Serial.println(stateNames[newState]);

  currentState = newState;
  stateStartTime = millis();
}

// ══════════════════════════════════════════════════════════════════
//  MOTOR DURDURMA
// ══════════════════════════════════════════════════════════════════
void motorlariDurdur() {
  esc_m1.writeMicroseconds(1500);
  esc_m2.writeMicroseconds(1500);
  esc_m3.writeMicroseconds(1500);
  esc_m4.writeMicroseconds(1500);
  esc_m5.writeMicroseconds(1500);
  esc_m6.writeMicroseconds(1500);
}

// ══════════════════════════════════════════════════════════════════
//  MOTORLARI SÜR (Roll PID uygulayarak)
// ══════════════════════════════════════════════════════════════════
void motorlariSur() {
  int final_m1 = pwm_m1;
  int final_m2 = pwm_m2;
  int final_m3 = pwm_m3;
  int final_m4 = pwm_m4;

  // Roll PID sadece dikey motorlara uygulanır
  int final_m5 = pwm_m5 - (int)roll_output;
  int final_m6 = pwm_m6 + (int)roll_output;

  final_m5 = constrain(final_m5, 1000, 2000);
  final_m6 = constrain(final_m6, 1000, 2000);

  esc_m1.writeMicroseconds(final_m1);
  esc_m2.writeMicroseconds(final_m2);
  esc_m3.writeMicroseconds(final_m3);
  esc_m4.writeMicroseconds(final_m4);
  esc_m5.writeMicroseconds(final_m5);
  esc_m6.writeMicroseconds(final_m6);
}

// ══════════════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Wire.begin();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  pinMode(PIN_HALL_EFFECT, INPUT_PULLUP);

  // ESC'leri bağla
  esc_m1.setPeriodHertz(50); esc_m1.attach(PIN_M1, 1000, 2000);
  esc_m2.setPeriodHertz(50); esc_m2.attach(PIN_M2, 1000, 2000);
  esc_m3.setPeriodHertz(50); esc_m3.attach(PIN_M3, 1000, 2000);
  esc_m4.setPeriodHertz(50); esc_m4.attach(PIN_M4, 1000, 2000);
  esc_m5.setPeriodHertz(50); esc_m5.attach(PIN_M5, 1000, 2000);
  esc_m6.setPeriodHertz(50); esc_m6.attach(PIN_M6, 1000, 2000);

  motorlariDurdur();
  delay(2000); // ESC arming

  // CAN (opsiyonel)
  if (CAN.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK) {
    CAN.setMode(MCP_NORMAL);
  }

  // IMU
  if (IMU.begin(0x6B)) {
    imuOK = true;
    Serial.println("[INIT] IMU OK");
  } else {
    imuOK = false;
    Serial.println("[INIT] IMU HATASI - Roll PID devre disi");
  }

  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(-200, 200);

  // Görev başlangıç zamanı
  stateStartTime = millis();

  Serial.println("════════════════════════════════════════════════");
  Serial.println("  AnaROV Standalone Video Gorev Firmware v1.0");
  Serial.println("  Jetson YOK — Tum gorev Deneyap uzerinde");
  Serial.println("════════════════════════════════════════════════");
  Serial.print  ("  Guc Siniri  : %"); Serial.println(POWER_LIMIT);
  Serial.print  ("  Bekleme     : "); Serial.print(DURATION_WAIT);   Serial.println("s");
  Serial.print  ("  Dalis       : "); Serial.print(DURATION_DIVE);   Serial.println("s");
  Serial.print  ("  Duz gidis   : "); Serial.print(DURATION_FORWARD);Serial.println("s");
  Serial.print  ("  90° donus   : "); Serial.print(DURATION_TURN_90);Serial.println("s");
  Serial.print  ("  Daire       : "); Serial.print(DURATION_CIRCLE); Serial.println("s");
  Serial.print  ("  Yuzey cikis : "); Serial.print(DURATION_SURFACE);Serial.println("s");
  Serial.println("════════════════════════════════════════════════");
  Serial.println("🚀 Gorev basliyor...");
}

// ══════════════════════════════════════════════════════════════════
//  ANA DÖNGÜ
// ══════════════════════════════════════════════════════════════════
void loop() {
  // --- 1. DONANIMSAL KORUMA (Manyetik Şalter) ---
  if (digitalRead(PIN_HALL_EFFECT) == LOW) {
    motorlariDurdur();
    return;
  }

  // --- 2. IMU OKUMA + PID ---
  if (imuOK) {
    float accelX = IMU.readFloatAccelX();
    float accelY = IMU.readFloatAccelY();
    float accelZ = IMU.readFloatAccelZ();
    roll_input = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
    rollPID.Compute();
  } else {
    roll_output = 0; // IMU yoksa PID devre dışı
  }

  // --- 3. DURUM MAKİNESİ ---
  float elapsed = (millis() - stateStartTime) / 1000.0; // Saniye cinsinden

  switch (currentState) {

    // ── DURUM 0: BEKLEME (su yüzeyinde, motorlar nötr) ────────
    case STATE_WAIT:
      thrusterMix(0, 0, 0, 0);
      if (elapsed >= DURATION_WAIT) {
        changeState(STATE_DIVE);
      }
      break;

    // ── DURUM 1: DALIŞ (agresif aşağı itki) ──────────────────
    case STATE_DIVE:
      thrusterMix(0, 0, 0, -SPEED_DIVE);
      if (elapsed >= DURATION_DIVE) {
        changeState(STATE_FORWARD_1);
      }
      break;

    // ── DURUM 2: 1. DÜZLÜK (ileri + derinlik koruma) ─────────
    case STATE_FORWARD_1:
      thrusterMix(SPEED_FORWARD, 0, 0, -SPEED_HOLD);
      if (elapsed >= DURATION_FORWARD) {
        changeState(STATE_TURN_1);
      }
      break;

    // ── DURUM 3: SAĞA 90° DÖN ────────────────────────────────
    case STATE_TURN_1:
      thrusterMix(0, 0, SPEED_TURN, -SPEED_HOLD);  // Pozitif yaw = sağa
      if (elapsed >= DURATION_TURN_90) {
        changeState(STATE_FORWARD_2);
      }
      break;

    // ── DURUM 4: 2. DÜZLÜK ───────────────────────────────────
    case STATE_FORWARD_2:
      thrusterMix(SPEED_FORWARD, 0, 0, -SPEED_HOLD);
      if (elapsed >= DURATION_FORWARD) {
        changeState(STATE_CIRCLE);
      }
      break;

    // ── DURUM 5: 360° DAİRE (ileri + yaw = orbital daire) ────
    //    Min 1m çap, min 1 tur kendi etrafında
    case STATE_CIRCLE:
      thrusterMix(CIRCLE_FWD, 0, CIRCLE_YAW, -SPEED_HOLD);
      if (elapsed >= DURATION_CIRCLE) {
        changeState(STATE_TURN_AFTER_CIRCLE);
      }
      break;

    // ── DURUM 6: DAİRE SONRASI 90° SAĞA DÖN ─────────────────
    case STATE_TURN_AFTER_CIRCLE:
      thrusterMix(0, 0, SPEED_TURN, -SPEED_HOLD);
      if (elapsed >= DURATION_TURN_90) {
        changeState(STATE_FORWARD_3);
      }
      break;

    // ── DURUM 7: 3. DÜZLÜK ───────────────────────────────────
    case STATE_FORWARD_3:
      thrusterMix(SPEED_FORWARD, 0, 0, -SPEED_HOLD);
      if (elapsed >= DURATION_FORWARD) {
        changeState(STATE_TURN_2);
      }
      break;

    // ── DURUM 8: SAĞA 90° DÖN ────────────────────────────────
    case STATE_TURN_2:
      thrusterMix(0, 0, SPEED_TURN, -SPEED_HOLD);
      if (elapsed >= DURATION_TURN_90) {
        changeState(STATE_FORWARD_4);
      }
      break;

    // ── DURUM 9: 4. DÜZLÜK (başlangıca dönüş) ───────────────
    case STATE_FORWARD_4:
      thrusterMix(SPEED_FORWARD, 0, 0, -SPEED_HOLD);
      if (elapsed >= DURATION_FORWARD) {
        changeState(STATE_SURFACE);
      }
      break;

    // ── DURUM 10: YÜZEYE ÇIKIŞ ───────────────────────────────
    case STATE_SURFACE:
      thrusterMix(0, 0, 0, SPEED_SURFACE);  // Pozitif = yukarı
      if (elapsed >= DURATION_SURFACE) {
        changeState(STATE_FINISHED);
      }
      break;

    // ── DURUM 11: GÖREV TAMAMLANDI ───────────────────────────
    case STATE_FINISHED:
      thrusterMix(0, 0, 0, 0);  // Tüm motorlar nötr
      motorlariDurdur();
      if (elapsed < 1.0) {
        Serial.println("════════════════════════════════════════");
        Serial.println("🎯 GOREV TAMAMLANDI!");
        Serial.println("════════════════════════════════════════");
      }
      return;  // Loop'a devam etme
  }

  // --- 4. MOTORLARI SÜR (Roll PID ile) ---
  if (currentState != STATE_FINISHED) {
    motorlariSur();
  }

  // --- 5. DEBUG ÇIKTISI (500ms aralıkla) ---
  unsigned long now = millis();
  if (now - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = now;
    Serial.print("[");
    Serial.print(stateNames[currentState]);
    Serial.print("] t=");
    Serial.print(elapsed, 1);
    Serial.print("s | M1:");
    Serial.print(pwm_m1);
    Serial.print(" M2:");
    Serial.print(pwm_m2);
    Serial.print(" M3:");
    Serial.print(pwm_m3);
    Serial.print(" M4:");
    Serial.print(pwm_m4);
    Serial.print(" M5:");
    Serial.print(pwm_m5);
    Serial.print(" M6:");
    Serial.print(pwm_m6);
    if (imuOK) {
      Serial.print(" | Roll:");
      Serial.print(roll_input, 1);
    }
    Serial.println();
  }
}
