from flask import Flask, Response
import cv2
import threading
import time

app = Flask(__name__)
cap = None
camera_id = -1

import numpy as np

# Otomatik Kamera Bulucu ve Pembe Ekran (MJPG) Çözücü
for i in range(4):
    # V4L2 backend'ini zorluyoruz
    temp_cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
    if temp_cap.isOpened():
        # Jetson'da YUV formatı bazen pembe/yeşil ekran verir. Zorla MJPG istiyoruz:
        temp_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        temp_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        temp_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        ret, frame = temp_cap.read()
        if ret and frame is not None:
            # Görüntü tamamen düz bir renk mi? (Örn: Dümdüz pembe veya yeşil)
            # Standart sapması (varyansı) 1.0'dan küçükse bu gerçek bir kamera değil, sahte/boş bir kanaldır.
            if np.std(frame) < 2.0:
                print(f"⚠️ /dev/video{i} SAHTE (PEMBE/BOŞ) KANAL ÇIKTI. Atlanıyor...")
                temp_cap.release()
                continue
                
            cap = temp_cap
            camera_id = i
            print(f"✅ GERÇEK KAMERA BULUNDU: /dev/video{camera_id}")
            break
    temp_cap.release()

if cap is None:
    print("❌ HİÇBİR KAMERA BULUNAMADI! Lütfen USB kablosunu kontrol edin.")
    # Kodun çökmemesi için sahte kamera aç
    cap = cv2.VideoCapture(-1)

def generate_frames():
    while True:
        success, frame = cap.read()
        if not success or frame is None:
            time.sleep(0.1)
            continue
        
        # Olası aşırı büyük çözünürlükleri ağdan geçebilsin diye ufaltıyoruz
        frame = cv2.resize(frame, (640, 480))
        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return f"✅ Jetson Kamera Yayini Aktif! (Video{camera_id}) Görüntü /video dizininde."

def run_server():
    print("🚀 KAMERA YAYINI BAŞLADI!")
    print("📡 Windows laptopunuzdan şu adrese bağlanın: http://<JETSON_IP>:5000/video")
    app.run(host='0.0.0.0', port=5000, threaded=True)

if __name__ == '__main__':
    run_server()
