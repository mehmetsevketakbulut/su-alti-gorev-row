from flask import Flask, Response
import cv2
import threading
import time

app = Flask(__name__)
cap = None
camera_id = -1

import numpy as np

print("🚀 [MANUEL YUYV] Pembe Ekran Düzeltici başlatılıyor...")
cap = cv2.VideoCapture(1)

# OpenCV'nin bozuk renk dönüştürücüsünü kapatıyoruz (Böylece bize ham pembe değil, YUYV verisi gelecek)
cap.set(cv2.CAP_PROP_CONVERT_RGB, False)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("❌ Kamera açılamadı! Lütfen kabloyu kontrol edin.")
    cap = cv2.VideoCapture(-1)

def generate_frames():
    while True:
        success, frame = cap.read()
        if not success or frame is None:
            import time
            time.sleep(0.1)
            continue
        
        # Ham YUYV verisini bizim kendi yöntemimizle düzgün renklere (BGR) çeviriyoruz
        try:
            # Gelen ham veri tek kanallı gibi görünür, onu YUV'dan BGR'a manuel çeviriyoruz
            if len(frame.shape) == 2 or frame.shape[2] == 1:
                frame = frame.reshape((720, 1280, 2)) # YUYV yapısı
                frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUYV)
        except Exception as e:
            pass # Eğer zaten BGR geldiyse veya boyut uymadıysa elleme
            
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
