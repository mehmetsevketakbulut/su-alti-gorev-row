#!/usr/bin/env python3
"""
Jetson Mesafe Sensörü Bağımsız Test Scripti (ROS2'den bağımsız)
Kullanım: python3 test_distance.py
"""

import serial
import time
import sys

# UART portu Jetson için genelde /dev/ttyTHS1 veya /dev/ttyTHS0'dır.
# Eğer USB-TTL dönüştürücü kullanıyorsanız /dev/ttyUSB0 olabilir.
PORT = '/dev/ttyTHS1'
BAUD_RATE = 9600

def main():
    print("=============================================")
    print(f"Mesafe Sensörü Testi Başlıyor...")
    print(f"Port: {PORT} | Baud: {BAUD_RATE}")
    print("Çıkmak için: CTRL+C")
    print("=============================================\n")

    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        print("✅ Port başarıyla açıldı. Veri bekleniyor...\n")
    except Exception as e:
        print(f"❌ Port AÇILAMADI! Hata: {e}")
        print("\nÖNERİ: İzin hatası (Permission denied) alıyorsanız şu komutu çalıştırın:")
        print(f"sudo chmod 666 {PORT}")
        sys.exit(1)

    try:
        while True:
            # 1. Byte oku (Başlangıç byte'ı 0xFF olmalı)
            byte = ser.read(1)
            
            if len(byte) == 0:
                continue # Veri yok
                
            if byte[0] == 0xFF:
                # Kalan 3 byte'ı oku (DATA_H, DATA_L, CHECKSUM)
                data = ser.read(3)
                
                if len(data) == 3:
                    data_h = data[0]
                    data_l = data[1]
                    checksum = data[2]

                    # Checksum (doğrulama) hesapla
                    expected_checksum = (0xFF + data_h + data_l) & 0xFF
                    
                    if checksum == expected_checksum:
                        # Mesafeyi hesapla
                        distance_mm = (data_h << 8) | data_l
                        distance_cm = distance_mm / 10.0
                        
                        # Terminale yazdır
                        print(f"📏 Ölçülen Mesafe: {distance_cm:.1f} cm", end='\r')
                    else:
                        print(f"\n⚠️ Veri bozuk (Checksum hatası)! Beklenen: {expected_checksum}, Gelen: {checksum}")
            
            time.sleep(0.01) # CPU'yu yormamak için çok kısa bekleme

    except KeyboardInterrupt:
        print("\n\n🛑 Test kullanıcı tarafından durduruldu.")
    finally:
        if ser.is_open:
            ser.close()
            print("Port kapatıldı.")

if __name__ == '__main__':
    main()
