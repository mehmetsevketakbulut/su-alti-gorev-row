#!/usr/bin/env python3
import time
try:
    import ms5837
except ImportError:
    print("HATA: ms5837 kütüphanesi bulunamadı. Lütfen kurun: pip3 install ms5837 smbus2")
    exit(1)

# Jetson'da I2C bus genelde 1 (veya 0, 8 olabilir, pinlere gore degisir)
I2C_BUS = 0 

print(f"MS5837 Basınç Sensörü Testi Başlıyor (I2C Bus: {I2C_BUS})...")

sensor = ms5837.MS5837_02BA(I2C_BUS)

if not sensor.init():
    print("HATA: Sensör başlatılamadı! Lütfen şunları kontrol edin:")
    print("1. Sensör Jetson'ın SDA ve SCL pinlerine doğru takılı mı?")
    print("2. Sensöre 3.3V güç gidiyor mu?")
    print("3. I2C Bus numarası doğru mu? (Terminalde 'i2cdetect -y 1' yazıp 0x76 adresini görebiliyor musunuz?)")
    exit(1)

print("Sensör başarıyla bulundu!")
print("Akışkan Yoğunluğu Tatlı Su olarak ayarlanıyor...")
sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)

print("Veri okunuyor... Çıkmak için CTRL+C'ye basın.\n")
print("-" * 50)

try:
    while True:
        if sensor.read():
            # Değerleri al
            basinc_mbar = sensor.pressure()
            sicaklik_c = sensor.temperature()
            derinlik_m = sensor.depth()
            
            print(f"Basınç: {basinc_mbar:.2f} mbar | Sıcaklık: {sicaklik_c:.2f} °C | Derinlik: {derinlik_m:.3f} m")
        else:
            print("Veri okuma hatası! Sensör bağlantısı kopmuş olabilir.")
            
        time.sleep(0.5)
        
except KeyboardInterrupt:
    print("\nTest sonlandırıldı.")
