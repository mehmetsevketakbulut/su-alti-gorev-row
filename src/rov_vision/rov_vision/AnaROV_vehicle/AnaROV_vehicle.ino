#include <ESP32Servo.h>
#include <mcp_can.h>
#include <SPI.h>
#include <Wire.h> 
#include <Deneyap_6EksenAtaletselOlcumBirimi.h>
#include <PID_v1.h>
#include <math.h>

// ══════════════════════════════════════════════════════════════════
//  DONANIM PİN TANIMLARI (Deneyap Kart — ESP32)
// ══════════════════════════════════════════════════════════════════

// --- SPI / CAN ---
#define SPI_CS 21      // D4

// --- Motor ESC Pinleri ---
#define PIN_M1 2       // D9  (Ön Sağ)
#define PIN_M2 12      // D13 (Ön Sol)
#define PIN_M3 23      // D0  (Arka Sağ)
#define PIN_M4 14      // D14 (Arka Sol)
#define PIN_M5 13      // D12 (Dikey SOL)
#define PIN_M6 22      // D1  (Dikey SAĞ)

// --- Sensörler ---
#define PIN_HALL_EFFECT 0  // D8  (Manyetik Şalter)

// Jetson USB üzerinden bağlanacak (Serial objesi kullanılacak)

// ══════════════════════════════════════════════════════════════════
//  DONANIM NESNELERİ
// ══════════════════════════════════════════════════════════════════
MCP_CAN CAN(SPI_CS);
LSM6DSM IMU; 

Servo esc_m1, esc_m2, esc_m3, esc_m4, esc_m5, esc_m6;

// ══════════════════════════════════════════════════════════════════
//  ZAMANLAMA VE GÜVENLİK
// ══════════════════════════════════════════════════════════════════
unsigned long sonVeriZamani = 0;
const int FAILSAFE_SURESI = 1000;  // 1 saniye veri gelmezse motorları durdur

// Debug yazdırma hızı
unsigned long sonYazdirmaZamani = 0;
const int YAZDIRMA_ARALIGI = 500;  // 500ms'de bir debug yazdır

// Paket sayaçları (debug için)
unsigned long alinanPaketSayisi = 0;
unsigned long hataliPaketSayisi = 0;

// Kill switch durumu
bool killSwitchAktif = false;

// ══════════════════════════════════════════════════════════════════
//  PID DEĞİŞKENLERİ (Roll / Yalpama Ekseni İçin)
// ══════════════════════════════════════════════════════════════════
double roll_input, roll_output;
double roll_setpoint = 0.0; 
double Kp = 1.5, Ki = 0.0, Kd = 0.25; 
PID rollPID(&roll_input, &roll_output, &roll_setpoint, Kp, Ki, Kd, DIRECT);

int base_pwm_m1 = 1500, base_pwm_m2 = 1500, base_pwm_m3 = 1500;
int base_pwm_m4 = 1500, base_pwm_m5 = 1500, base_pwm_m6 = 1500;

// ══════════════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════════════
void setup() {
  // --- Jetson USB Serial Bağlantısı ---
  Serial.begin(115200);
  
  Wire.begin(); 

  // ESP32Servo zamanlayıcılarını tahsis edelim (Daha kararlı çalışması için)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  pinMode(PIN_HALL_EFFECT, INPUT_PULLUP);

  esc_m1.setPeriodHertz(50); // Standart ESC frekansı
  esc_m2.setPeriodHertz(50);
  esc_m3.setPeriodHertz(50);
  esc_m4.setPeriodHertz(50);
  esc_m5.setPeriodHertz(50);
  esc_m6.setPeriodHertz(50);

  esc_m1.attach(PIN_M1, 1000, 2000); 
  esc_m2.attach(PIN_M2, 1000, 2000); 
  esc_m3.attach(PIN_M3, 1000, 2000);
  esc_m4.attach(PIN_M4, 1000, 2000); 
  esc_m5.attach(PIN_M5, 1000, 2000); 
  esc_m6.attach(PIN_M6, 1000, 2000);
  
  motorlariDurdur();
  delay(2000); // ESC silahlanma (arming) süresi

  if(CAN.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK) {
    CAN.setMode(MCP_NORMAL);
    Serial.println("[INIT] CAN Bus baslatildi.");
  } else {
    Serial.println("[INIT] CAN Bus HATASI! (Gorev etkilenmez)");
  }

  if (IMU.begin(0x6B)) {
    Serial.println("[INIT] IMU (LSM6DSM) baslatildi.");
  } else {
    Serial.println("[INIT] IMU HATASI! Roll PID devre disi.");
  }

  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(-200, 200); 

  Serial.println("══════════════════════════════════════════");
  Serial.println("  AnaROV Motor Kontrolcusu v2.1");
  Serial.println("  Jetson Baglantisi: USB (Serial)");
  Serial.println("  Protokol: A,m1,m2,m3,m4,m5,m6,btn,kp,kd");
  Serial.println("══════════════════════════════════════════");
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

  // --- 2. SERİ HABERLEŞME (Jetson USB — Serial üzerinden) ---
  // Jetson'dan gelen paket: A,m1,m2,m3,m4,m5,m6,btn,kp,kd\n
  // Değerler: -100 ile +100 arası yüzdelik dilimler
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim(); // Satır sonu karakterlerini temizle (\r, boşluk vs.)
    
    if (data.startsWith("A,")) {
      int v[9];
      // sscanf ile veriyi güvenli ve tek satırda ayrıştırıyoruz.
      // Tam olarak 9 tane tamsayı (integer) gelip gelmediğini kontrol ediyoruz.
      int parsed = sscanf(data.c_str(), "A,%d,%d,%d,%d,%d,%d,%d,%d,%d", 
                          &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8]);
      
      // SADECE eksiksiz bir paket (9 eleman) geldiyse motorlara ata
      if (parsed == 9) {
        sonVeriZamani = millis();
        alinanPaketSayisi++;
        
        // --- Kill Switch Kontrolü (v[6] = btn_kapat) ---
        if (v[6] == 1) {
          killSwitchAktif = true;
          motorlariDurdur();
          return;
        } else {
          killSwitchAktif = false;
        }
        
        // Gelen değerler -100 ile +100 arası yüzdelik değerler.
        // map fonksiyonuyla PWM'e çeviriyoruz. (int8_t cast kaldırıldı)
        base_pwm_m1 = map(constrain(v[0], -100, 100), -100, 100, 1000, 2000);
        base_pwm_m2 = map(constrain(v[1], -100, 100), -100, 100, 1000, 2000);
        base_pwm_m3 = map(constrain(v[2], -100, 100), -100, 100, 1000, 2000);
        base_pwm_m4 = map(constrain(v[3], -100, 100), -100, 100, 1000, 2000);
        base_pwm_m5 = map(constrain(v[4], -100, 100), -100, 100, 1000, 2000);
        
        // M6 motoru ters map ediliyor (fiziksel montaj tersi)
        base_pwm_m6 = map(constrain(v[5], -100, 100), -100, 100, 2000, 1000);
        
        // PID Parametrelerini güncelle (v[7] = kp*100, v[8] = kd*100)
        Kp = (double)v[7] / 100.0;
        Kd = (double)v[8] / 100.0;
        rollPID.SetTunings(Kp, Ki, Kd);
      } else {
        hataliPaketSayisi++;
      }
    }
  }

  // --- 3. KILL SWITCH AKTİFSE MOTORLARI DURDUR ---
  if (killSwitchAktif) {
    motorlariDurdur();
    return;
  }

  // --- 4. IMU OKUMA VE PID HESAPLAMA (YALPAMA/ROLL) ---
  float accelX = IMU.readFloatAccelX();
  float accelY = IMU.readFloatAccelY();
  float accelZ = IMU.readFloatAccelZ();
  
  roll_input = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  rollPID.Compute(); 

  // --- 5. MOTOR SÜRÜŞÜ ---
  unsigned long mevcutZaman = millis();

  if (mevcutZaman - sonVeriZamani <= FAILSAFE_SURESI) {
    
    int final_m1 = base_pwm_m1;
    int final_m2 = base_pwm_m2;
    int final_m3 = base_pwm_m3;
    int final_m4 = base_pwm_m4;

    int final_m5 = base_pwm_m5 - roll_output; 
    int final_m6 = base_pwm_m6 + roll_output;

    final_m5 = constrain(final_m5, 1000, 2000);
    final_m6 = constrain(final_m6, 1000, 2000);

    esc_m1.writeMicroseconds(final_m1);
    esc_m2.writeMicroseconds(final_m2);
    esc_m3.writeMicroseconds(final_m3);
    esc_m4.writeMicroseconds(final_m4);
    esc_m5.writeMicroseconds(final_m5);
    esc_m6.writeMicroseconds(final_m6);

    // Not: Jetson artık bu hatta dinliyor olduğu için yoğun debug çıktısını kapalı tutmakta
    // veya sadece belirli durumlarda kullanmakta fayda var.
    /*
    if (mevcutZaman - sonYazdirmaZamani >= YAZDIRMA_ARALIGI) {
      sonYazdirmaZamani = mevcutZaman;
      Serial.print("[OK] PKT:");
      // ... debug loglar
    }
    */
    
  } else {
    motorlariDurdur();
    
    // Failsafe debug (500ms aralıkla) - Jetson tarafında görebilirsiniz
    if (mevcutZaman - sonYazdirmaZamani >= YAZDIRMA_ARALIGI) {
      sonYazdirmaZamani = mevcutZaman;
      Serial.print("[FAILSAFE] Veri yok! Son paket: ");
      Serial.print((mevcutZaman - sonVeriZamani) / 1000);
      Serial.print("s once | Toplam alınan: ");
      Serial.print(alinanPaketSayisi);
      Serial.print(" | Hatali: ");
      Serial.println(hataliPaketSayisi);
    }
  }
}

// ══════════════════════════════════════════════════════════════════
//  MOTOR DURDURMA (Tüm ESC'leri nötr 1500µs'ye al)
// ══════════════════════════════════════════════════════════════════
void motorlariDurdur() {
  esc_m1.writeMicroseconds(1500);
  esc_m2.writeMicroseconds(1500);
  esc_m3.writeMicroseconds(1500);
  esc_m4.writeMicroseconds(1500);
  esc_m5.writeMicroseconds(1500);
  esc_m6.writeMicroseconds(1500);
}