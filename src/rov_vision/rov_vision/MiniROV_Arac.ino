#include <ESP32Servo.h>
#include <mcp_can.h>
#include <SPI.h>
#include <Deneyap_6EksenAtaletselOlcumBirimi.h>
#include <math.h>

// --- PİN TANIMLARI ---
#define SPI_CS D4       
#define PIN_ARSOL D1    
#define PIN_ARSAG D12   
#define PIN_ONSOL D13   
#define PIN_ONSAG D14   

#define PIN_KAPATMA D15   
#define PIN_ISIK_MINI D9  

#define MIN_GUC 1236
#define MAX_GUC 1764

MCP_CAN CAN(SPI_CS);
LSM6DSM IMU;

Servo onsag, onsol, arsag, arsol;

double Setpoint, Input, Output;
double Kp = 1.5; // Basit P kontrolcü katsayısı
float komp_ileri = 0.25; 
float komp_batma = 0.15; 

volatile bool acilDurum = false;
bool yazilimsalKilit = false; 

// YENİ: İletişim kopmasını algılamak için zamanlayıcı
unsigned long sonCanZamani = 0; 

void IRAM_ATTR acilKapatmaISR() { acilDurum = true; }

void motorlariDurdur() {
  onsag.writeMicroseconds(1500); onsol.writeMicroseconds(1500);
  arsag.writeMicroseconds(1500); arsol.writeMicroseconds(1500);
}

void sistemiKilitle() {
  motorlariDurdur();
}

void setup() {
  Serial.begin(115200);

  // CAN Modülünü başlat
  CAN.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ);
  CAN.setMode(MCP_NORMAL);

  ledcSetup(9, 5000, 8); 
  ledcAttachPin(PIN_ISIK_MINI, 9);
  ledcWrite(9, 0);

  onsag.attach(PIN_ONSAG, 1000, 2000); onsol.attach(PIN_ONSOL, 1000, 2000);
  arsag.attach(PIN_ARSAG, 1000, 2000); arsol.attach(PIN_ARSOL, 1000, 2000);
  
  pinMode(PIN_KAPATMA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_KAPATMA), acilKapatmaISR, FALLING);
  
  // ==========================================
  // YENİ: ESC RESET / ARMING SÜRECİ
  // ==========================================
  motorlariDurdur();
  delay(3000); // ESC'lerin bipleyip kurulması için 3 saniye nötr bekle!
  motorlariDurdur();

  IMU.begin(0x6A);
  Setpoint = 0.0;
}

void loop() {
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];

  float accX = IMU.readFloatAccelX();
  float accZ = IMU.readFloatAccelZ();
  Input = atan2(accX, accZ) * 180.0 / PI;
  
  // Basit P kontrolcü (PID_v1 kütüphanesi olmadan)
  double error = Setpoint - Input;
  Output = constrain(error * Kp, -300, 300);

  // --- CAN BUS DİNLEME ---
  if(CAN.checkReceive() == CAN_MSGAVAIL) {
    CAN.readMsgBuf(&rxId, &len, rxBuf);
    
    // Geçerli bir paket aldık, zamanlayıcıyı sıfırla!
    sonCanZamani = millis(); 

    if (rxId == 0x04 && len == 1) {
      yazilimsalKilit = (rxBuf[0] == 1); 
    }
    else if (rxId == 0x07 && len == 1) { 
      ledcWrite(9, map(rxBuf[0], 0, 100, 0, 255));
    }
    else if (rxId == 0x02 && len == 8) {
      int y1_batma_ham = (rxBuf[0] << 8) | rxBuf[1];
      int x1_roll_ham  = (rxBuf[2] << 8) | rxBuf[3];
      int x2_donme_ham = (rxBuf[4] << 8) | rxBuf[5];
      int y2_ileri_ham = (rxBuf[6] << 8) | rxBuf[7];

      int y1_cikis = 1500 - y1_batma_ham;
      int x2_donme = x2_donme_ham - 1500;
      int y2_ileri = 1500 - y2_ileri_ham;

      Setpoint = map(x1_roll_ham, 1060, 1940, -45, 45);

      x2_donme = (x2_donme / 500.0) * (x2_donme / 500.0) * (x2_donme / 500.0) * 500.0 * 0.5;
      y2_ileri = y2_ileri * 0.7;

      int destek_ileri = y1_cikis * komp_ileri;
      int destek_batma = y2_ileri * komp_batma;

      int onsolPWM = constrain(1500 + y1_cikis + destek_batma + (int)Output, MIN_GUC, MAX_GUC);
      int onsagPWM = constrain(1500 - y1_cikis - destek_batma - (int)Output, MIN_GUC, MAX_GUC);
      int arsolPWM = constrain(1500 + y2_ileri + destek_ileri + x2_donme, MIN_GUC, MAX_GUC);
      int arsagPWM = constrain(1500 + y2_ileri + destek_ileri - x2_donme, MIN_GUC, MAX_GUC);

      if (acilDurum || yazilimsalKilit) {
        sistemiKilitle();
      } else {
        onsag.writeMicroseconds(onsagPWM); onsol.writeMicroseconds(onsolPWM);
        arsag.writeMicroseconds(arsagPWM); arsol.writeMicroseconds(arsolPWM);
      }
    }
  }

  // ==========================================
  // YENİ: İLETİŞİM ZAMAN AŞIMI (FAILSAFE)
  // ==========================================
  // 1 saniyeden uzun süredir AnaROV'dan CAN paketi gelmediyse veya kilitliyse
  if (acilDurum || yazilimsalKilit || (millis() - sonCanZamani > 1000)) {
    sistemiKilitle();
  }
}