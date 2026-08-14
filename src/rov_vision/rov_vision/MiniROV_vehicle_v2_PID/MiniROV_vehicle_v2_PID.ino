#include <ESP32Servo.h>

// ══════════════════════════════════════════════════════════════════
//  DONANIM PİN TANIMLARI (Senin Koddaki Birebir Aynı Pinler)
// ══════════════════════════════════════════════════════════════════
#define PIN_ONSAG D1
#define PIN_ONSOL D12
#define PIN_ARSAG D13
#define PIN_ARSOL D14 

// Güç kesici / Röle pini (OUTPUT)
#define PIN_KAPATMA D15 

Servo onsag, onsol, arsag, arsol;

// ══════════════════════════════════════════════════════════════════
//  HIZ AYARI
// ══════════════════════════════════════════════════════════════════
// 1500 = Nötr (Durma)
// 1600 = Hafif ileri, 1800 = Hızlı ileri
const int TEST_HIZI = 1600; 

void setup() {
  Serial.begin(115200);
  Serial.println("\n[SİSTEM] 4 Motorlu Test Kodu Başlatılıyor...");

  // Kapatma (Röle) pini ayarları - Sistemi açık tutmak için LOW yapıyoruz
  pinMode(PIN_KAPATMA, OUTPUT);
  digitalWrite(PIN_KAPATMA, LOW); 

  // ESP32Servo zamanlayıcıları
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Standart ESC Frekansı
  onsag.setPeriodHertz(50);
  onsol.setPeriodHertz(50);
  arsag.setPeriodHertz(50);
  arsol.setPeriodHertz(50);

  // Motorları pinlere bağla (Sinyal aralığı 1000-2000 mikrosaniye)
  onsag.attach(PIN_ONSAG, 1000, 2000); 
  onsol.attach(PIN_ONSOL, 1000, 2000);
  arsag.attach(PIN_ARSAG, 1000, 2000); 
  arsol.attach(PIN_ARSOL, 1000, 2000);
  
  // ESC'leri başlatmak (Arming) için önce hepsine "DUR" (1500) sinyali yolluyoruz.
  motorlariDurdur();
  Serial.println("[BİLGİ] ESC'ler silahlandırılıyor (Arming). Lütfen 5 saniye bekleyin...");
  delay(5000); 
  
  Serial.println("[BİLGİ] SİLAHLANDIRMA TAMAM! Motorlar dönmeye başlıyor...");
}

void loop() {
  // --- MOTORLARI DÖNDÜR ---
  onsag.writeMicroseconds(TEST_HIZI);
  onsol.writeMicroseconds(TEST_HIZI);
  arsag.writeMicroseconds(TEST_HIZI);
  arsol.writeMicroseconds(TEST_HIZI);

  // Çalıştığını görmek için Seri Port'a bilgi yazdır
  Serial.print("Tüm motorlar dönüyor. Güncel PWM sinyali: ");
  Serial.println(TEST_HIZI);
  
  delay(500); // Serial monitör okunabilsin diye yarım saniye gecikme
}

// ══════════════════════════════════════════════════════════════════
//  TÜM MOTORLARI DURDURMA FONKSİYONU
// ══════════════════════════════════════════════════════════════════
void motorlariDurdur() {
  onsag.writeMicroseconds(1500);
  onsol.writeMicroseconds(1500);
  arsag.writeMicroseconds(1500);
  arsol.writeMicroseconds(1500);
}