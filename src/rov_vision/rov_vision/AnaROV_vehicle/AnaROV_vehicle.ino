#include <ESP32Servo.h>
#include <mcp_can.h>
#include <SPI.h>
#include <Wire.h> 
#include <Deneyap_6EksenAtaletselOlcumBirimi.h>
#include <PID_v1.h>
#include <math.h>

// --- Donanım Pinleri (Gerçek GPIO Numaraları Kullanıldı) ---
#define SPI_CS 21      // D4
#define PIN_M1 2       // D9  (Ön Sağ)
#define PIN_M2 12      // D13 (Ön Sol)
#define PIN_M3 23      // D0  (Arka Sağ)
#define PIN_M4 14      // D14 (Arka Sol)
#define PIN_M5 13      // D12 (Dikey SOL)
#define PIN_M6 22      // D1  (Dikey SAĞ)
#define PIN_HALL_EFFECT 0  // D8  (Manyetik Şalter)

MCP_CAN CAN(SPI_CS);
LSM6DSM IMU; 

Servo esc_m1, esc_m2, esc_m3, esc_m4, esc_m5, esc_m6;

unsigned long sonVeriZamani = 0;
const int FAILSAFE_SURESI = 1000; 

// Ekrana yazdırma hızını kontrol etmek için yeni zamanlayıcı[cite: 3]
unsigned long sonYazdirmaZamani = 0;
const int YAZDIRMA_ARALIGI = 200; // 200 milisaniyede bir (saniyede 5 kez) yazdır

// --- PID Değişkenleri (Roll / Yalpama Ekseni İçin) ---
double roll_input, roll_output;
double roll_setpoint = 0.0; 
double Kp = 1.5, Ki = 0.0, Kd = 0.25; 
PID rollPID(&roll_input, &roll_output, &roll_setpoint, Kp, Ki, Kd, DIRECT);

int base_pwm_m1 = 1500, base_pwm_m2 = 1500, base_pwm_m3 = 1500;
int base_pwm_m4 = 1500, base_pwm_m5 = 1500, base_pwm_m6 = 1500;

void setup() {
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
    Serial.println("CAN Baslatildi.");
  } else {
    Serial.println("CAN Hatasi!");
    // while(1) { delay(1000); } 
  }

  if (IMU.begin(0x6B)) {
    Serial.println("IMU Baslatildi.");
  } else {
    Serial.println("IMU Hatasi!");
  }

  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(-200, 200); 
}

void loop() {
  // --- 1. DONANIMSAL KORUMA ---
  if (digitalRead(PIN_HALL_EFFECT) == LOW) { 
    motorlariDurdur();
    return; 
  }

  // --- 2. SERİ HABERLEŞME (Jetson TX/RX Doğrudan Bağlantısı) ---
  // Jetson'dan gelen paket: A,m1,m2,m3,m4,m5,m6,btn,kp,kd\n
  // Değerler artık doğrudan PWM (1100-1900 arası)
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim(); // Satır sonu karakterlerini temizle
    
    if (data.startsWith("A,")) {
      int v[9];
      // sscanf ile veriyi çok daha güvenli ve tek satırda ayrıştırıyoruz. 
      // Tam olarak 9 tane tamsayı (integer) gelip gelmediğini kontrol ediyoruz.
      int parsed = sscanf(data.c_str(), "A,%d,%d,%d,%d,%d,%d,%d,%d,%d", 
                          &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8]);
      
      // SADECE ve SADECE eksiksiz bir paket (9 eleman) geldiyse motorlara ata!
      // Aksi halde veri bozuk gelmiş demektir (örn: kablo gürültüsü) ve reddedilir.
      if (parsed == 9) {
        sonVeriZamani = millis(); 
        
        // Gelen değerler tekrar -100 ile +100 arası yüzdelik değerler.
        // Eski sistemdeki gibi map fonksiyonuyla PWM'e çeviriyoruz.
        base_pwm_m1 = map((int8_t)v[0], -100, 100, 1000, 2000);
        base_pwm_m2 = map((int8_t)v[1], -100, 100, 1000, 2000);
        base_pwm_m3 = map((int8_t)v[2], -100, 100, 1000, 2000);
        base_pwm_m4 = map((int8_t)v[3], -100, 100, 1000, 2000);
        base_pwm_m5 = map((int8_t)v[4], -100, 100, 1000, 2000);
        
        // M6 motoru eski kodda olduğu gibi ters map ediliyor
        base_pwm_m6 = map((int8_t)v[5], -100, 100, 2000, 1000);
        
        // PID Parametrelerini güncelle (v[7] = kp, v[8] = kd)
        Kp = (double)v[7] / 100.0;
        Kd = (double)v[8] / 100.0;
        rollPID.SetTunings(Kp, Ki, Kd);
      }
    }
  }

  // --- 3. IMU OKUMA VE PID HESAPLAMA (YALPAMA/ROLL) ---
  float accelX = IMU.readFloatAccelX();
  float accelY = IMU.readFloatAccelY();
  float accelZ = IMU.readFloatAccelZ();
  
  roll_input = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  rollPID.Compute(); 

  // --- 4. MOTOR SÜRÜŞÜ ---
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

    // --- EKRANA YAZDIRMA (Gecikmesiz Blok) ---
    if (mevcutZaman - sonYazdirmaZamani >= YAZDIRMA_ARALIGI) {
      sonYazdirmaZamani = mevcutZaman;
      Serial.print("M1(OnSag): "); Serial.print(final_m1);
      Serial.print(" | M2(OnSol): "); Serial.print(final_m2);
      Serial.print(" | M3(ArkSag): "); Serial.print(final_m3);
      Serial.print(" | M4(ArkSol): "); Serial.print(final_m4);
      Serial.print(" | M5(DikSol): "); Serial.print(final_m5);
      Serial.print(" | M6(DikSag): "); Serial.println(final_m6);
    }
    
  } else {
    motorlariDurdur();
  }
}

void motorlariDurdur() {
  esc_m1.writeMicroseconds(1500);
  esc_m2.writeMicroseconds(1500);
  esc_m3.writeMicroseconds(1500);
  esc_m4.writeMicroseconds(1500);
  esc_m5.writeMicroseconds(1500);
  esc_m6.writeMicroseconds(1500);
}