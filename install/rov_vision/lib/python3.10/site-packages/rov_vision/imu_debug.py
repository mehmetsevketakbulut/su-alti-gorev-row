#!/usr/bin/env python3
import time
import sys

print("=== BNO085 IMU HATA AYIKLAMA ARACI ===")

# 1. Kütüphaneleri kontrol et
try:
    from adafruit_extended_bus import ExtendedI2C
    import board
    import busio
    import bitbangio
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_bno08x import (
        BNO_REPORT_ROTATION_VECTOR,
        BNO_REPORT_GAME_ROTATION_VECTOR,
        BNO_REPORT_ACCELEROMETER
    )
except ImportError as e:
    print(f"❌ Kütüphane eksik: {e}")
    sys.exit(1)

I2C_BUS = 7
I2C_ADDR = 0x4A

def test_sensor(i2c_bus_obj, name):
    print(f"\n[{name}] üzerinden BNO085 test ediliyor...")
    try:
        bno = BNO08X_I2C(i2c_bus_obj, address=I2C_ADDR)
        print("✅ I2C bağlantısı başarılı, sensör bulundu!")
        
        # Bekle ki sensör kendine gelsin
        time.sleep(1.0)
        
        print("-> ACCELEROMETER özelliği açılıyor...")
        bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        print("✅ ACCELEROMETER başarıyla açıldı!")
        
        print("-> GAME_ROTATION_VECTOR özelliği açılıyor...")
        try:
            bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
            print("✅ GAME_ROTATION_VECTOR başarıyla açıldı!")
        except Exception as e:
            print(f"⚠️ GAME_ROTATION_VECTOR açılamadı: {e}")
            
            print("-> ROTATION_VECTOR özelliği açılıyor (alternatif)...")
            bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            print("✅ ROTATION_VECTOR başarıyla açıldı!")

        # Veri okumayı dene
        for i in range(5):
            accel = bno.acceleration
            print(f"   İvme verisi [{i+1}/5]: {accel}")
            time.sleep(0.1)

        return True
    except ValueError as e:
        print(f"❌ Sensör bulunamadı veya I2C hatası: {e}")
        return False
    except RuntimeError as e:
        print(f"❌ Çalışma zamanı hatası (Clock Stretching/Timeout olabilir): {e}")
        return False
    except Exception as e:
        print(f"❌ Beklenmeyen hata: {e}")
        return False

# Yöntem 1: ExtendedI2C (Hardware I2C via /dev/i2c-7)
print("\n--- YÖNTEM 1: Linux Hardware I2C (/dev/i2c-7) ---")
try:
    hw_i2c = ExtendedI2C(I2C_BUS)
    test_sensor(hw_i2c, "ExtendedI2C(7)")
except Exception as e:
    print(f"ExtendedI2C başlatılamadı: {e}")

# Yöntem 2: BitBang I2C (Software I2C)
print("\n--- YÖNTEM 2: Software I2C (BitBang I2C) ---")
print("Not: Bu yöntem I2C clock stretching (zaman uzatma) sorunlarını çözer.")
try:
    sw_i2c = bitbangio.I2C(board.SCL, board.SDA)
    test_sensor(sw_i2c, "bitbangio.I2C")
except Exception as e:
    print(f"Software I2C başlatılamadı: {e}")
    print("Not: BNO085 Jetson üzerinde donanımsal I2C-7'ye bağlıysa, GPIO pinleri standart SCL/SDA olmayabilir.")

print("\n=============================================")
print("Eğer iki yöntem de başarısız oluyorsa, sorun %90 I2C Clock Stretching'dir.")
print("BNO085'in Jetson donanımsal I2C denetleyicisiyle uyumsuzluğu meşhurdur.")
print("Çözüm: I2C hızını (baudrate) Linux üzerinden 100kHz'e veya 400kHz'e sabitlemek veya donanımsal I2C yerine UART (Seri) modunu kullanmaktır.")
