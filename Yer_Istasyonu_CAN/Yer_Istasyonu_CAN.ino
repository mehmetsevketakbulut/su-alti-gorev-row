#include <SPI.h>
#include <mcp2515.h>

// Topside Deneyap CAN Module CS Pin (D4 on Deneyap)
#define PIN_CS_CAN D4 

struct can_frame msg_eksen;
MCP2515 mcp2515(PIN_CS_CAN);

unsigned long lastSendTime = 0;

// Varsayılan PWM değerleri (Nötr = 1500)
int dive_pwm   = 1500;
int strafe_pwm = 1500;
int yaw_pwm    = 1500;
int fwd_pwm    = 1500;
int fs_mini    = 0; // 0 = Armed, 1 = Kilitli (Failsafe)

void setup() {
  Serial.begin(115200);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.println("YER ISTASYONU CAN MODULU HAZIR!");
}

void loop() {
  // Python Arayüzünden (yer_istasyonu.py) Seri Port üzerinden veri bekle
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    
    // Format: M,fwd,strafe,dive,yaw,fs_mini
    if (data.startsWith("M,")) {
      int v[5];
      if (sscanf(data.c_str(), "M,%d,%d,%d,%d,%d", &v[0], &v[1], &v[2], &v[3], &v[4]) == 5) {
        
        // Gelen -100 to 100 değerlerini 1000-2000 PWM aralığına çevir
        fwd_pwm    = map(constrain(v[0], -100, 100), -100, 100, 1000, 2000);
        strafe_pwm = map(constrain(v[1], -100, 100), -100, 100, 1000, 2000);
        dive_pwm   = map(constrain(v[2], -100, 100), -100, 100, 1000, 2000);
        yaw_pwm    = map(constrain(v[3], -100, 100), -100, 100, 1000, 2000);
        fs_mini    = v[4];
      }
    }
  }

  // Mini ROV'un kilitlenmemesi (Failsafe'e girmemesi) için saniyede 20 kere sürekli paket yolla (50ms)
  if (millis() - lastSendTime >= 50) {
    lastSendTime = millis();

    // 1. MOTOR EKSEN KOMUTLARI (0x02)
    msg_eksen.can_id = 0x02; msg_eksen.can_dlc = 8;
    msg_eksen.data[0] = highByte(dive_pwm); msg_eksen.data[1] = lowByte(dive_pwm);
    msg_eksen.data[2] = highByte(strafe_pwm); msg_eksen.data[3] = lowByte(strafe_pwm);
    msg_eksen.data[4] = highByte(yaw_pwm); msg_eksen.data[5] = lowByte(yaw_pwm);
    msg_eksen.data[6] = highByte(fwd_pwm); msg_eksen.data[7] = lowByte(fwd_pwm);
    mcp2515.sendMessage(&msg_eksen);

    // 2. FAILSAFE / YAZILIMSAL KİLİT KOMUTU (0x04)
    struct can_frame msg_kapat;
    msg_kapat.can_id = 0x04; msg_kapat.can_dlc = 1;
    msg_kapat.data[0] = (byte)fs_mini;
    mcp2515.sendMessage(&msg_kapat);
  }
}
