#!/usr/bin/env python3
import serial
import time
import sys

PORT = 'COM14'
BAUD_RATE = 9600

def main():
    print(f"Sensör dinleniyor... Port: {PORT} | Hız: {BAUD_RATE}")
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=0.1)
    except Exception as e:
        print(f"Hata: {e}")
        sys.exit(1)

    try:
        while True:
            data = ser.read(ser.in_waiting or 1)
            if data:
                for b in data:
                    if b != 0:  # 0x00 boşluklarını gizleyelim
                        print(f"Gelen Değer: {b} mm ({b/10.0} cm)")
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()
