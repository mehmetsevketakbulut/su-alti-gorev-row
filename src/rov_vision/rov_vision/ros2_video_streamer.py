import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import time
from flask import Flask, Response

# Flask Uygulaması
app = Flask(__name__)

# Global değişkenler
latest_frame = None
frame_lock = threading.Lock()

class Ros2VideoStreamer(Node):
    """
    ROS 2 üzerinden gelen işlenmiş görüntüleri (/line_follower/debug_image)
    veya ham görüntüleri (/camera/image_raw) alıp Flask üzerinden Windows 
    Yer İstasyonuna (Ground Station) yayınlayan köprü düğümü.
    """
    def __init__(self):
        super().__init__('ros2_video_streamer')
        
        # Otonom sürüşteki işlenmiş görüntüyü dinle (çizgiler, metrikler vs.)
        # Eğer otonom sürüş kapalıysa, sadece siyah ekran veya uyarı vermemesi için
        # aslında hem image_raw hem de debug_image dinleyip, debug_image varsa onu, 
        # yoksa image_raw'ı da basabiliriz. Ama şimdilik doğrudan debug_image dinliyoruz.
        self.declare_parameter('image_topic', '/line_follower/debug_image')
        topic = self.get_parameter('image_topic').value
        
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info(f"🌐 ROS 2 Video Streamer Başlatıldı! Dinlenen Topic: {topic}")
        self.get_logger().info(f"📡 Windows laptopunuzdan bağlanın: http://<JETSON_IP>:5000/video")
        
    def image_callback(self, msg):
        global latest_frame
        try:
            # Gelen ROS 2 mesajını OpenCV formatına (BGR) çevir
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            with frame_lock:
                latest_frame = cv_image
        except Exception as e:
            self.get_logger().error(f"Görüntü dönüştürme hatası: {e}")

def generate_frames():
    """
    Flask için sonsuz döngüde kare üreten jeneratör fonksiyon.
    """
    global latest_frame
    
    while True:
        with frame_lock:
            if latest_frame is None:
                frame = None
            else:
                frame = latest_frame.copy()
                
        if frame is None:
            # Henüz ROS 2'den görüntü gelmediyse bekle
            time.sleep(0.1)
            continue
            
        # Olası aşırı büyük çözünürlükleri ağdan geçebilsin diye ufaltıyoruz
        # (Gerçi video_publisher zaten 640x480 basıyor ama güvenlik amaçlı)
        frame = cv2.resize(frame, (640, 480))
        
        # JPEG olarak kodla
        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        
        if not ret:
            continue
            
        frame_bytes = buffer.tobytes()
        
        # MJPEG formatında yayınla
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def ros2_thread(node):
    """ROS 2 spin döngüsünü ayrı bir thread'de çalıştırır."""
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = Ros2VideoStreamer()
    
    # ROS 2 dinleme işlemini arka plana at
    thread = threading.Thread(target=ros2_thread, args=(node,), daemon=True)
    thread.start()
    
    # Flask sunucusunu ana thread'de başlat
    try:
        # debug=False olmalı, aksi takdirde Flask ana thread'de hata verebilir
        app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
