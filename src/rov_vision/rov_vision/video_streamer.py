import subprocess
import time
from flask import Flask, Response

app = Flask(__name__)

def find_working_camera():
    # video0'dan video3'e kadar çalışan kamerayı bul
    for i in range(4):
        dev = f"/dev/video{i}"
        # FFMPEG'in bu cihaza erişip erişemediğini test et
        cmd = ['ffmpeg', '-y', '-f', 'v4l2', '-i', dev, '-vframes', '1', '-f', 'null', '-']
        ret = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if ret.returncode == 0:
            return dev
    return None

def generate_frames():
    dev = find_working_camera()
    if not dev:
        print("❌ HİÇBİR KAMERA BULUNAMADI! Kabloyu kontrol edin.")
        # Çökmemesi için boş yayın
        while True:
            time.sleep(1)
            yield b''

    print(f"✅ GERÇEK KAMERA BULUNDU (FFMPEG): {dev}")
    
    # OpenCV'nin tüm bug'larını (pembe ekran, beyaz ekran, çökmeler) aşmak için 
    # doğrudan sistemin kalbi olan FFMPEG'i kullanıyoruz!
    command = [
        'ffmpeg',
        '-f', 'v4l2',
        '-i', dev,
        '-s', '1280x720',
        '-c:v', 'mjpeg',
        '-q:v', '5', # Yüksek kalite
        '-f', 'image2pipe',
        '-'
    ]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    
    buffer = b''
    while True:
        chunk = process.stdout.read(8192)
        if not chunk:
            print("❌ FFMPEG yayını koptu!")
            break
        buffer += chunk
        
        # JPEG başlangıç (FF D8) ve bitiş (FF D9) baytlarını bul
        start = buffer.find(b'\xff\xd8')
        end = buffer.find(b'\xff\xd9')
        
        if start != -1 and end != -1 and end > start:
            # Tam bir JPEG karesi yakalandı
            jpg = buffer[start:end+2]
            # Okunan kısmı buffer'dan sil
            buffer = buffer[end+2:]
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')

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
