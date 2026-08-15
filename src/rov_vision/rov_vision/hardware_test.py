#!/usr/bin/env python3
import time
import serial
import math
import sys

# I2C ve Sensör Kütüphaneleri
try:
    from adafruit_extended_bus import ExtendedI2C as I2C
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_ACCELEROMETER
    IMU_OK = True
except ImportError:
    print("BNO085 kütüphanesi yok. Kurmak için: pip3 install adafruit-circuitpython-bno08x adafruit-extended-bus")
    IMU_OK = False

try:
    import ms5837
    PRESSURE_OK = True
except ImportError:
    print("MS5837 kütüphanesi yok. Kurmak için: pip3 install ms5837 smbus2")
    PRESSURE_OK = False

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(t0, t1))
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.degrees(math.asin(t2))
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(t3, t4))
    return roll, pitch, yaw

def main():
    print("=== DONANIM ENTEGRASYON TESTİ ===")
    
    # 1. Deneyap Bağlantısı
    ser = None
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
        ser.setDTR(False)
        ser.setRTS(False)
        print("✅ Deneyap bağlandı (/dev/ttyUSB0)")
    except Exception as e:
        print(f"❌ Deneyap bağlantı hatası: {e}")

    # 2. IMU Bağlantısı
    bno = None
    if IMU_OK:
        try:
            i2c_imu = I2C(7)
            bno = BNO08X_I2C(i2c_imu, address=0x4A)
            bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
            bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            print("✅ IMU (BNO085) I2C-7 üzerinden bağlandı")
            time.sleep(0.1) # Sensörün uyanması için bekle
        except Exception as e:
            print(f"❌ IMU bağlantı hatası: {e}")

    # 3. Basınç Sensörü Bağlantısı
    pressure_sensor = None
    if PRESSURE_OK:
        try:
            temp_sensor = ms5837.MS5837_30BA(1) # i2c-1 portunda
            init_success = False
            for _ in range(5):
                if temp_sensor.init():
                    init_success = True
                    break
                time.sleep(0.5)
                
            if init_success:
                temp_sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
                pressure_sensor = temp_sensor
                print("✅ Basınç Sensörü (MS5837) I2C-1 üzerinden bağlandı")
            else:
                print("❌ Basınç Sensörü başlatılamadı! (Bağlantıyı kontrol edin)")
        except Exception as e:
            print(f"❌ Basınç Sensörü hatası: {e}")

    print("\n--- Sensör Verileri Okunuyor (Çıkmak için CTRL+C) ---")
    
    try:
        while True:
            # Deneyap'a durması için dummy veri gönder (canlı tutmak için ve haberleşmeyi tetiklemek için)
            if ser is not None:
                # Format: A,m1,m2,m3,m4,m5,m6,btn,kp,kd\n
                packet = "A,0,0,0,0,0,0,0,0,0\n"
                ser.write(packet.encode('utf-8'))

            # Deneyap'tan gelen veriyi oku
            deneyap_data = ""
            if ser is not None and ser.in_waiting > 0:
                deneyap_data = ser.readline().decode('utf-8', errors='ignore').strip()

            # IMU Oku
            roll, pitch, yaw = 0.0, 0.0, 0.0
            accel_x, accel_y, accel_z = 0.0, 0.0, 0.0
            if bno is not None:
                try:
                    quat = bno.game_quaternion
                    if quat and quat[0] is not None:
                        roll, pitch, yaw = euler_from_quaternion(quat[0], quat[1], quat[2], quat[3])
                    
                    acc = bno.acceleration
                    if acc and acc[0] is not None:
                        accel_x, accel_y, accel_z = acc
                except Exception as e:
                    print(f"[IMU HATASI]: {e}")

            # Basınç Oku
            depth = 0.0
            if pressure_sensor is not None:
                try:
                    if pressure_sensor.read():
                        depth = pressure_sensor.depth() * 100.0 # cm'ye çevir
                except Exception as e:
                    print(f"[BASINÇ HATASI]: {e}")

            print(f"IMU(R:{roll:5.1f} P:{pitch:5.1f} | AX:{accel_x:5.1f} AY:{accel_y:5.1f} AZ:{accel_z:5.1f}) | Der: {depth:5.1f}cm | Deneyap: {deneyap_data}")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nTest bitiriliyor...")
        if ser is not None:
            # Çıkarken motorları kesin kapat
            ser.write("A,0,0,0,0,0,0,0,0,0\n".encode('utf-8'))
            ser.close()

if __name__ == '__main__':
    main()
