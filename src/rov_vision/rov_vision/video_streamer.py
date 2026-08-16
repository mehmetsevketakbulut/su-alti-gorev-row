from flask import Flask, Response
import cv2
import threading
import time

app = Flask(__name__)
cap = None
camera_id = -1

import numpy as np

import numpy as np

print("🚀 Akıllı Kamera ve Çözünürlük Tarayıcı Başlatılıyor...")
cap = None
camera_id = -1

# USB Dönüştürücüler genellikle sadece tek bir çözünürlüğü (Örn 1920x1080) destekler.
# Eğer yanlış çözünürlük sorarsak gerçek kamera (video0) çöker, kod gidip sahte kamerayı (video1) açar.
# Bu yüzden her port için olası tüm çözünürlükleri deneyeceğiz!
resolutions_to_try = [(1920, 1080), (1280, 720), (640, 480)]

for i in range(4):
    for w, h in resolutions_to_try:
        temp_cap = cv2.VideoCapture(i)
        if temp_cap.isOpened():
            temp_cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            temp_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            
            # Görüntüyü oku
            ret, frame = temp_cap.read()
            
            # Hem görüntü var mı, hem de SAHTE (bembeyaz/yemyeşil/pembe) değil mi diye kontrol et
            # Standart sapma > 10.0 ise bu gerçek dünyayı gören bir kameradır!
            if ret and frame is not None and np.std(frame) > 10.0:
                print(f"✅ GERÇEK KAMERA BULUNDU: /dev/video{i} (Çözünürlük: {w}x{h})")
                cap = temp_cap
                camera_id = i
                break
            
            temp_cap.release()
    
    if cap is not None:
        break

if cap is None:
    print("❌ Kamera bulunamadı, sahte yayın başlatılıyor...")
    cap = cv2.VideoCapture(-1)

def generate_frames():
    while True:
        success, frame = cap.read()
        if not success or frame is None:
            import time
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
