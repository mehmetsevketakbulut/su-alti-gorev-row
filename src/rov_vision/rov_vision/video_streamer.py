from flask import Flask, Response
import cv2
import threading
import time

app = Flask(__name__)
cap = None
camera_id = -1

import numpy as np

# Jetson'da USB Dönüştürücüler (YUYV formatı) OpenCV ile direkt açıldığında PEMBE/YEŞİL glitch verir.
# Bunu çözmenin tek yolu GStreamer kullanmaktır.
gst_pipeline = (
    "v4l2src device=/dev/video1 ! "
    "video/x-raw, width=1280, height=720 ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink drop=1"
)

print("🚀 [GSTREAMER] Pembe Ekran Düzeltici (YUYV->BGR) başlatılıyor...")
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

if cap.isOpened():
    camera_id = 1
    print("✅ GERÇEK KAMERA BULUNDU (GStreamer): /dev/video1")
else:
    print("❌ GStreamer ile açılamadı, standart metoda dönülüyor...")
    # Yedek plan
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

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
