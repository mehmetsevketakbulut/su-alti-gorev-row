#include <ESP32Servo.h>
#include <mcp_can.h>
#include <SPI.h>
#include <Deneyap_6EksenAtaletselOlcumBirimi.h>
#include <PID_v1.h>
#include <math.h>

LSM6DSM IMU; 

#define SPI_CS D4
MCP_CAN CAN(SPI_CS);

Servo onsag, onsol, arsag, arsol;

// GÜVENLİ VE ÇAKIŞMASIZ PİNLER
#define PIN_ONSAG D1
#define PIN_ONSOL D12
#define PIN_ARSAG D13
#define PIN_ARSOL D14 
#define PIN_KAPATMA D15 

// %60 GÜÇ SINIRLARI
#define MIN_GUC 1236
#define MAX_GUC 1764

// --- GLOBAL DEĞİŞKENLER (Canlı Değiştirilebilir) ---
double Setpoint, Input, Output;
double Kp = 2.5, Ki = 0.0, Kd = 0.5;
float komp_ileri = 0.25; // Çapraz kompanzasyon - Arka motorlar için
float komp_batma = 0.15; // Çapraz kompanzasyon - Ön motorlar için

PID rollPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);
  delay(1000); 

  if(CAN.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK) {
    CAN.setMode(MCP_NORMAL);
    Serial.println("Arac Ici Sistem Hazir!");
  } else {
    while(1) { Serial.println("Arac Ici CAN Hatasi!"); delay(1000); }
  }

  // 1. ÖNCE MOTORLARI BAŞLAT VE EMI GÜRÜLTÜSÜNÜ ATLAT
  onsag.attach(PIN_ONSAG, 1000, 2000); 
  onsol.attach(PIN_ONSOL, 1000, 2000);
  arsag.attach(PIN_ARSAG, 1000, 2000); 
  arsol.attach(PIN_ARSOL, 1000, 2000);
  
  pinMode(PIN_KAPATMA, OUTPUT);
  digitalWrite(PIN_KAPATMA, LOW); 
  
  onsag.writeMicroseconds(1500);
  onsol.writeMicroseconds(1500);
  arsag.writeMicroseconds(1500);
  arsol.writeMicroseconds(1500);
  
  Serial.println("ESC'ler aciliyor, 5 saniye bekleniyor...");
  delay(5000); 

  // 2. ORTAM SAKİNLEŞTİKTEN SONRA IMU'YU UYANDIR
  if (IMU.begin(0x6A) == 0) { 
    Serial.println("✅ BASARILI: IMU Baglandi!");
  } else {
    Serial.println("❌ HATA: IMU bulunamadi!");
  }
  
  // PID BAŞLANGIÇ AYARLARI
  Setpoint = 0.0; 
  rollPID.SetMode(AUTOMATIC); 
  rollPID.SetOutputLimits(-300, 300); 
  rollPID.SetSampleTime(20); 
}

void loop() {
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];

  // 1. İVMEÖLÇER OKUMA VE AUTO-RECOVERY
  float accX = IMU.readFloatAccelX();
  float accZ = IMU.readFloatAccelZ();

  if (accX == 0.00 && accZ == 0.00) {
      Serial.println("DIKKAT: IMU Koptu! Yeniden Baslatiliyor...");
      IMU.begin(0x6A); 
      delay(50); 
      accX = IMU.readFloatAccelX(); 
      accZ = IMU.readFloatAccelZ();
  }
  
  // Yatma açısını bul ve PID'ye ver
  Input = atan2(accX, accZ) * 180.0 / PI; 
  rollPID.Compute(); 
  
  static unsigned long son_yazdirma = 0;
  if(millis() - son_yazdirma > 500) { 
      Serial.print("Kp: "); Serial.print(Kp);
      Serial.print(" | Kd: "); Serial.print(Kd);
      Serial.print(" | Setpoint: "); Serial.print(Setpoint);
      Serial.print(" | Aci: "); Serial.print(Input);
      Serial.print(" | K-Ileri: "); Serial.print(komp_ileri);
      Serial.print(" | K-Batma: "); Serial.println(komp_batma);
      son_yazdirma = millis();
  }

  // 2. CAN BUS VERİ OKUMA
  if(CAN.checkReceive() == CAN_MSGAVAIL) {
    CAN.readMsgBuf(&rxId, &len, rxBuf);

    // --- KAPATMA PAKETİ (0x04) ---
    if (rxId == 0x04 && len == 1) {
      if(rxBuf[0] == 1) {
        onsag.writeMicroseconds(1500);
        onsol.writeMicroseconds(1500);
        arsag.writeMicroseconds(1500);
        arsol.writeMicroseconds(1500);
        digitalWrite(PIN_KAPATMA, HIGH); 
        delay(500);
        digitalWrite(PIN_KAPATMA, LOW);
        while(1) { delay(100); } 
      }
    }

    // --- CANLI PID GÜNCELLEMESİ (0x05) - 4 Byte ---
    if (rxId == 0x05 && len == 4) {
      int kp_raw = (rxBuf[0] << 8) | rxBuf[1];
      int kd_raw = (rxBuf[2] << 8) | rxBuf[3];
      Kp = kp_raw / 100.0;
      Kd = kd_raw / 100.0;
      rollPID.SetTunings(Kp, Ki, Kd);
    }
    
    // --- CANLI KOMPANZASYON KATSAYILARI (0x06) - 4 Byte ---
    if (rxId == 0x06 && len == 4) {
      int ki_raw = (rxBuf[0] << 8) | rxBuf[1];
      int kb_raw = (rxBuf[2] << 8) | rxBuf[3];
      komp_ileri = ki_raw / 100.0;
      komp_batma = kb_raw / 100.0;
    }
    
    // --- SÜRÜŞ VE ANGLE MODE (0x02) - 8 Byte ---
    if (rxId == 0x02 && len == 8) {
      int y1_batma_ham = (rxBuf[0] << 8) | rxBuf[1];
      int x1_roll_ham  = (rxBuf[2] << 8) | rxBuf[3]; 
      int x2_donme_ham = (rxBuf[4] << 8) | rxBuf[5];
      int y2_ileri_ham = (rxBuf[6] << 8) | rxBuf[7];

      int y1_cikis = 1500 - y1_batma_ham; 
      int x2_donme = x2_donme_ham - 1500; 
      int y2_ileri = 1500 - y2_ileri_ham; 

      // ----------------------------------------------------
      // ANGLE MODE: Joystick verisini (-45 ile +45) dereceye çevirip PID hedefine yazar
      Setpoint = map(x1_roll_ham, 1060, 1940, -45, 45);
      // ----------------------------------------------------

      float donme_hassasiyeti = 0.5; 
      float ileri_hassasiyeti = 0.7; 
      float donme_normalize = x2_donme / 500.0; 
      x2_donme = (donme_normalize * donme_normalize * donme_normalize) * 500.0 * donme_hassasiyeti;
      y2_ileri = y2_ileri * ileri_hassasiyeti;

      // CROSS-COUPLING (Çapraz Kompanzasyon) Destek İtkileri
      int destek_ileri = y1_cikis * komp_ileri;
      int destek_batma = y2_ileri * komp_batma;

      // KİNEMATİK DAĞILIM (Aynalı Pervane + Çapraz Denge + Angle Mode PID)
      int onsolPWM = 1500 + y1_cikis + destek_batma + (int)Output; 
      int onsagPWM = 1500 - y1_cikis - destek_batma - (int)Output; // Sağ motor (-) ile tersine çevrildi
      
      int arsolPWM = 1500 + y2_ileri + destek_ileri + x2_donme; 
      int arsagPWM = 1500 + y2_ileri + destek_ileri - x2_donme; 

      onsagPWM = constrain(onsagPWM, MIN_GUC, MAX_GUC);
      onsolPWM = constrain(onsolPWM, MIN_GUC, MAX_GUC);
      arsagPWM = constrain(arsagPWM, MIN_GUC, MAX_GUC);
      arsolPWM = constrain(arsolPWM, MIN_GUC, MAX_GUC);

      onsag.writeMicroseconds(onsagPWM);
      onsol.writeMicroseconds(onsolPWM);
      arsag.writeMicroseconds(arsagPWM);
      arsol.writeMicroseconds(arsolPWM);
    }
  }
}