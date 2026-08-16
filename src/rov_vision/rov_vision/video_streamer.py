from flask import Flask, Response
import cv2
import threading

app = Flask(__name__)
# BlueOS'te /dev/video1 genellikle USB web kamerasıdır
camera_id = 1 
cap = cv2.VideoCapture(camera_id)

if not cap.isOpened():
    print(f"❌ KULLANILAMIYOR: /dev/video{camera_id}. Deneniyor: /dev/video0...")
    camera_id = 0
    cap = cv2.VideoCapture(camera_id)

def generate_frames():
    while True:
        success, frame = cap.read()
        if not success:
            break
        else:
            # Görüntüyü küçültüp ağda kasmasını engelliyoruz
            frame = cv2.resize(frame, (640, 480))
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video')
def video_feed():
    # MJPEG formatında yayın başlatır
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return "✅ Jetson Kamera Yayini Aktif! Görüntü /video dizininde."

def run_server():
    print("🚀 KAMERA YAYINI BAŞLADI!")
    print(f"📡 Windows laptopunuzdan şu adrese bağlanın: http://<JETSON_IP>:5000/video")
    app.run(host='0.0.0.0', port=5000, threaded=True)

if __name__ == '__main__':
    run_server()
