import serial
import time

# Jetson 40-pin header üzerindeki UART1 portu
ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=2)
time.sleep(1)

print("Jetson'dan Deneyap'a 'PING' gönderiliyor...")
ser.write(b"PING\n") # Mesajı gönderiyoruz

yanit = ser.readline().decode('utf-8').strip()

if yanit == "PONG":
    print("SUCCESS: Deneyap'tan 'PONG' cevabı geldi! İletişim tamam.")
else:
    print(f"HATA: Yanıt alınamadı veya hatalı yanıt geldi: '{yanit}'")

ser.close()