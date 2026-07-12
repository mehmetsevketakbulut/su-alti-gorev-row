#include <mcp_can.h>
#include <SPI.h>

#define SPI_CS D4
MCP_CAN CAN(SPI_CS);

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  
  if(CAN.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK) {
    CAN.setMode(MCP_NORMAL);
  } else {
    while(1) { Serial.println("CAN Modulu Hatasi!"); delay(1000); }
  }
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    // --- ANAROV KONTROL BLOĞU ---
    // Format: A,m1,m2,m3,m4,m5,m6,btn_kapat,kp_int,kd_int
    if (data.startsWith("A,")) {
      int c1 = data.indexOf(',');
      int c2 = data.indexOf(',', c1 + 1);
      int c3 = data.indexOf(',', c2 + 1);
      int c4 = data.indexOf(',', c3 + 1);
      int c5 = data.indexOf(',', c4 + 1);
      int c6 = data.indexOf(',', c5 + 1);
      int c7 = data.indexOf(',', c6 + 1);
      int c8 = data.indexOf(',', c7 + 1);
      int c9 = data.indexOf(',', c8 + 1);

      if (c1 > 0 && c9 > 0) {
        int m1 = data.substring(c1 + 1, c2).toInt();
        int m2 = data.substring(c2 + 1, c3).toInt();
        int m3 = data.substring(c3 + 1, c4).toInt();
        int m4 = data.substring(c4 + 1, c5).toInt();
        int m5 = data.substring(c5 + 1, c6).toInt();
        int m6 = data.substring(c6 + 1, c7).toInt();
        int btn_kapat = data.substring(c7 + 1, c8).toInt();
        int kp_int = data.substring(c8 + 1, c9).toInt();
        int kd_int = data.substring(c9 + 1).toInt();

        // 1. HAT: Motor Sürüş ve Yazılımsal Kapatma (ID: 0x10) - 7 Byte
        byte anaRovData[7] = { 
          (byte)m1, (byte)m2, (byte)m3, (byte)m4, (byte)m5, (byte)m6, (byte)btn_kapat 
        };
        CAN.sendMsgBuf(0x10, 0, 7, anaRovData); 

        // 2. HAT: PID Ayarları (ID: 0x11) - 4 Byte
        byte pidData[4] = {
          highByte(kp_int), lowByte(kp_int), highByte(kd_int), lowByte(kd_int)
        };
        CAN.sendMsgBuf(0x11, 0, 4, pidData);
      }
    } 
    // --- MINIROV KONTROL BLOĞU (ORİJİNAL) ---
    else {
      int c1 = data.indexOf(',');
      int c2 = data.indexOf(',', c1 + 1);
      int c3 = data.indexOf(',', c2 + 1);
      int c4 = data.indexOf(',', c3 + 1);
      int c5 = data.indexOf(',', c4 + 1); 
      int c6 = data.indexOf(',', c5 + 1);
      int c7 = data.indexOf(',', c6 + 1); 
      int c8 = data.indexOf(',', c7 + 1);

      if (c1 > 0 && c8 > 0) {
        int val_y1 = data.substring(0, c1).toInt();
        int val_x1 = data.substring(c1 + 1, c2).toInt(); 
        int val_x2 = data.substring(c2 + 1, c3).toInt();
        int val_y2 = data.substring(c3 + 1, c4).toInt();
        int btn_kapat = data.substring(c4 + 1, c5).toInt();
        int kp_int = data.substring(c5 + 1, c6).toInt();
        int kd_int = data.substring(c6 + 1, c7).toInt();
        int komp_ileri_int = data.substring(c7 + 1, c8).toInt();
        int komp_batma_int = data.substring(c8 + 1).toInt();

        byte eksenData[8] = {
          highByte(val_y1), lowByte(val_y1), 
          highByte(val_x1), lowByte(val_x1),
          highByte(val_x2), lowByte(val_x2), 
          highByte(val_y2), lowByte(val_y2)
        };
        CAN.sendMsgBuf(0x02, 0, 8, eksenData);

        byte kapatData[1] = {(byte)btn_kapat};
        CAN.sendMsgBuf(0x04, 0, 1, kapatData);
        
        byte pidDataMin[4] = {highByte(kp_int), lowByte(kp_int), highByte(kd_int), lowByte(kd_int)};
        CAN.sendMsgBuf(0x05, 0, 4, pidDataMin);
        
        byte kompData[4] = {highByte(komp_ileri_int), lowByte(komp_ileri_int), highByte(komp_batma_int), lowByte(komp_batma_int)};
        CAN.sendMsgBuf(0x06, 0, 4, kompData);
      }
    }
  }
}