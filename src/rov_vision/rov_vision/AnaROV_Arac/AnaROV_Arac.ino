#include <ESP32Servo.h>
#include <mcp2515.h>
#include <SPI.h>

// ------------------- KULLANICININ VERDIGI YENI PINLER -------------------
#define SPI_CS D4       
#define PIN_M1 DAC1      
#define PIN_M2 DAC2       
#define PIN_M3 D15      
#define PIN_M4 D14       
#define PIN_M5 D12       
#define PIN_M6 A5      

#define PIN_SERVO A4    // Kamera tilt servo sinyal
#define PIN_MIKNATIS D0 // Elektromiknatis MOSFET gate
#define PIN_ROLE D1     // Kamera rolesi (LOW=Ana, HIGH=Mini)
#define PIN_ISIK_ANA SDA// Ana ROV aydinlatma MOSFET gate (D10)
#define PIN_SISTEM_KAPAT D13 // Guc kesme rolesi

#define PIN_HALL_OKUMA D8 // Hall sensor (varsa)
// ------------------------------------------------------------------------

MCP2515 mcp2515(SPI_CS);
Servo esc_m1, esc_m2, esc_m3, esc_m4, esc_m5, esc_m6;
Servo tiltServo;

double roll_input = 0.0; 
double roll_output = 0.0;
double Kp = 1.5; // Basit P kontrolcu sabiti

int base_pwm[6] = {1500, 1500, 1500, 1500, 1500, 1500};
int mevcut_servo = 45;
unsigned long sonVeriZamani = 0;
unsigned long sonTelemZamani = 0;
bool failsafe_aktif = true;
bool sonHallDurumu = HIGH; 

void motorlariDurdur() {
  esc_m1.writeMicroseconds(1500); esc_m2.writeMicroseconds(1500);
  esc_m3.writeMicroseconds(1500); esc_m4.writeMicroseconds(1500);
  esc_m5.writeMicroseconds(1500); esc_m6.writeMicroseconds(1500);
  base_pwm[0]=1500; base_pwm[1]=1500; base_pwm[2]=1500;
  base_pwm[3]=1500; base_pwm[4]=1500; base_pwm[5]=1500;
}

void anaSalteriIndir() {
  motorlariDurdur(); 
  digitalWrite(PIN_SISTEM_KAPAT, HIGH); 
  while(1) { delay(100); } 
}

void setup() {
  Serial.begin(115200);   
  Serial.setTimeout(10); 
  
  pinMode(PIN_ROLE, OUTPUT);
  digitalWrite(PIN_ROLE, LOW);
  
  pinMode(PIN_MIKNATIS, OUTPUT);
  digitalWrite(PIN_MIKNATIS, LOW);

  pinMode(PIN_HALL_OKUMA, INPUT_PULLUP); 
  sonHallDurumu = digitalRead(PIN_HALL_OKUMA);
  
  pinMode(PIN_SISTEM_KAPAT, OUTPUT);     
  digitalWrite(PIN_SISTEM_KAPAT, LOW);   

  ledcSetup(8, 5000, 8);
  ledcAttachPin(PIN_ISIK_ANA, 8);
  ledcWrite(8, 0); 

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);

  esc_m1.attach(PIN_M1, 1000, 2000); esc_m2.attach(PIN_M2, 1000, 2000); 
  esc_m3.attach(PIN_M3, 1000, 2000); esc_m4.attach(PIN_M4, 1000, 2000); 
  esc_m5.attach(PIN_M5, 1000, 2000); esc_m6.attach(PIN_M6, 1000, 2000);
  
  tiltServo.attach(PIN_SERVO, 500, 2400);
  tiltServo.write(mevcut_servo);
  
  motorlariDurdur();
  delay(3000); 
  motorlariDurdur();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void loop() {
  bool anlikHall = digitalRead(PIN_HALL_OKUMA);
  
  if (anlikHall != sonHallDurumu) {
    delay(50); 
    if (digitalRead(PIN_HALL_OKUMA) == anlikHall) { 
      sonHallDurumu = anlikHall;
    }
  }

  while (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    
    if (data.startsWith("A,")) {
      int v[12]; 
      if (sscanf(data.c_str(), "A,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", 
                          &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8], &v[9], &v[10], &v[11]) == 12) {
        
        sonVeriZamani = millis();
        failsafe_aktif = (v[6] == 1);
        
        digitalWrite(PIN_ROLE, v[7] == 1 ? HIGH : LOW);
        digitalWrite(PIN_MIKNATIS, v[8] == 1 ? HIGH : LOW); 
        ledcWrite(8, map(v[9], 0, 100, 0, 255));
        roll_input = (double)v[10];
        mevcut_servo = constrain(v[11], 0, 90);
        tiltServo.write(mevcut_servo);

        base_pwm[0] = map(constrain(v[0], -100, 100), -100, 100, 1000, 2000);
        base_pwm[1] = map(constrain(v[1], -100, 100), -100, 100, 1000, 2000);
        base_pwm[2] = map(constrain(v[2], -100, 100), -100, 100, 1000, 2000);
        base_pwm[3] = map(constrain(v[3], -100, 100), -100, 100, 1000, 2000);
        base_pwm[4] = map(constrain(v[4], -100, 100), -100, 100, 1000, 2000);
        base_pwm[5] = map(constrain(v[5], -100, 100), -100, 100, 2000, 1000); 
      }
    }
    else if (data.startsWith("M,")) {
      int v[6]; 
      if (sscanf(data.c_str(), "M,%d,%d,%d,%d,%d,%d", 
                          &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) == 6) {
        
        struct can_frame msg_eksen;
        msg_eksen.can_id = 0x02; msg_eksen.can_dlc = 8;
        msg_eksen.data[0] = highByte(v[0]); msg_eksen.data[1] = lowByte(v[0]);
        msg_eksen.data[2] = highByte(v[1]); msg_eksen.data[3] = lowByte(v[1]);
        msg_eksen.data[4] = highByte(v[2]); msg_eksen.data[5] = lowByte(v[2]);
        msg_eksen.data[6] = highByte(v[3]); msg_eksen.data[7] = lowByte(v[3]);
        mcp2515.sendMessage(&msg_eksen);
        
        struct can_frame msg_kapat;
        msg_kapat.can_id = 0x04; msg_kapat.can_dlc = 1;
        msg_kapat.data[0] = (byte)v[4];
        mcp2515.sendMessage(&msg_kapat);
        
        struct can_frame msg_isik;
        msg_isik.can_id = 0x07; msg_isik.can_dlc = 1;
        msg_isik.data[0] = (byte)v[5];
        mcp2515.sendMessage(&msg_isik);
      }
    }
  }

  if (millis() - sonVeriZamani > 1000) {
    failsafe_aktif = true;
    motorlariDurdur(); 
  } else {
    // Basit P kontrolcu (PID_v1 kutuphanesi olmadan)
    roll_output = constrain(roll_input * Kp, -200, 200); 
    
    esc_m1.writeMicroseconds(base_pwm[0]); esc_m2.writeMicroseconds(base_pwm[1]);
    esc_m3.writeMicroseconds(base_pwm[2]); esc_m4.writeMicroseconds(base_pwm[3]);
    esc_m5.writeMicroseconds(constrain(base_pwm[4] - roll_output, 1000, 2000));
    esc_m6.writeMicroseconds(constrain(base_pwm[5] + roll_output, 1000, 2000));
  }

  // YER ISTASYONU ICIN TELEMETRI GONDER (100 ms'de bir)
  if (millis() - sonTelemZamani > 100) {
      sonTelemZamani = millis();
      // Format: T,tilt,m1,m2,m3,m4,m5,m6,failsafe,mag,canerr
      Serial.print("T,"); Serial.print(mevcut_servo);
      Serial.print(","); Serial.print(base_pwm[0]);
      Serial.print(","); Serial.print(base_pwm[1]);
      Serial.print(","); Serial.print(base_pwm[2]);
      Serial.print(","); Serial.print(base_pwm[3]);
      Serial.print(","); Serial.print(constrain(base_pwm[4] - roll_output, 1000, 2000));
      Serial.print(","); Serial.print(constrain(base_pwm[5] + roll_output, 1000, 2000));
      Serial.print(","); Serial.print(failsafe_aktif ? 1 : 0);
      Serial.print(","); Serial.print(digitalRead(PIN_MIKNATIS) == HIGH ? 1 : 0);
      Serial.println(",0");
  }
}
 }
}