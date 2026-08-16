from flask import Flask, Response
import cv2
import threading
import time

app = Flask(__name__)
cap = None
camera_id = -1

import numpy as np

print("🚀 [V4L2 NATIVE] YUYV Format Zorlayıcı Başlatılıyor...")

# Dönüştürücülerin %99'u YUYV formatındadır ve V4L2 ile açılmalıdır.
# FFMPEG (8UC1 beyaz ekran) hatasını atlamak için V4L2'yi ZORLUYORUZ.
camera_id = 1
cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)

if cap.isOpened():
    # Kameranın aklını karıştırmamak için doğrudan YUYV formatı istiyoruz
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    ret, frame = cap.read()
    if ret and frame is not None:
        print(f"✅ V4L2 BAŞARILI: /dev/video{camera_id} (Çözünürlük: {frame.shape})")
    else:
        print(f"❌ V4L2 Okuma Hatası (DQBUF)! Alternatif deneniyor...")
        cap.release()
        cap = None
else:
    cap = None

if cap is None:
    print("⚠️ video1 V4L2 ile açılamadı. FFMPEG olmadan sadece cv2 ile deneniyor...")
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

def generate_frames():
    while True:
        success, frame = cap.read()
        if not success or frame is None:
            import time
            time.sleep(0.1)
            continue
            
        # RAW okuduğumuz için manuel renk çevirisi (Pembe/Beyaz ekran çözümü)
        try:
            if len(frame.shape) == 2 or frame.shape[2] == 1:
                h, w = frame.shape[0], frame.shape[1]
                # YUYV formatı genişliğin 2 katı byte içerir
                frame = frame.reshape((h, w//2, 2))
                frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUYV)
        except Exception as e:
            pass

        
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
