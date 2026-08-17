import cv2
import time
from flask import Flask, Response

app = Flask(__name__)

def find_working_camera():
    print("🚀 OpenCV Brute-Force Kamera Tarayıcısı Başlatılıyor...")
    for i in range(4):
        # 1. Deneme: V4L2 + MJPG (En stabil ve renkleri en doğru format)
        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            ret, frame = cap.read()
            if ret and frame is not None and len(frame.shape) == 3:
                print(f"✅ KAMERA BULUNDU: /dev/video{i} (V4L2 + MJPG - Kusursuz Renk)")
                return cap
            cap.release()
            
        # 2. Deneme: V4L2 + YUYV (DQBUF hatası verebilir, verirse atlar)
        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            ret, frame = cap.read()
            if ret and frame is not None and len(frame.shape) == 3:
                print(f"✅ KAMERA BULUNDU: /dev/video{i} (V4L2 + YUYV - Kusursuz Renk)")
                return cap
            cap.release()
            
        # 3. Deneme: OpenCV Default (Pembe veya Siyah/Beyaz verebilir ama ASLA çökmez)
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                print(f"⚠️ KAMERA BULUNDU: /dev/video{i} (OpenCV Default - Renkler bozuk olabilir)")
                return cap
            cap.release()
            
    return None

# Sunucu başlarken kamerayı 1 KERE bulup kilitler, Windows bağlanırken Timeout olmaz!
GLOBAL_CAP = find_working_camera()

def generate_frames():
    global GLOBAL_CAP
    if GLOBAL_CAP is None:
        print("❌ HİÇBİR KAMERA BULUNAMADI! Kabloyu kontrol edin.")
        while True:
            time.sleep(1)
            yield b''

    while True:
        ret, frame = GLOBAL_CAP.read()
        if not ret or frame is None:
            time.sleep(0.1)
            continue
            
        # Olası aşırı büyük çözünürlükleri ağdan geçebilsin diye ufaltıyoruz
        frame = cv2.resize(frame, (640, 480))
        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

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
