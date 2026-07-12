import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        
        # ROS2 Parametresi: Varsayılan "0" (Webcam), ama dosya yolu da verilebilir (örn: "test_video.mp4")
        self.declare_parameter('video_source', '0')
        video_source = self.get_parameter('video_source').value
        
        # Görüntüyü yayınlayacağımız ROS 2 kanalı (topic)
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Saniyede 20 kare (20 FPS) için 0.05 saniyelik zamanlayıcı
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.bridge = CvBridge()
        
        # Kamera veya Video Dosyası açma
        self.is_file = False
        try:
            # Eğer sayıysa (0, 1, 2) webcam olarak aç
            source_id = int(video_source)
            self.cap = cv2.VideoCapture(source_id)
            # V4L2 ve MJPG ayarları (Webcam için)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 20)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except ValueError:
            # Sayı değilse (harf/yol içeriyorsa) video dosyası olarak aç
            self.cap = cv2.VideoCapture(video_source)
            self.is_file = True

        if not self.cap.isOpened():
            self.get_logger().error(f"❌ Video kaynağı açılamadı: {video_source}")
            # Kamera açılamadıysa timer'ı durdur ki sürekli hata basmasın
            self.timer.cancel()
        else:
            if self.is_file:
                self.get_logger().info(f"🎞️ Video dosyasından yayın başladı: {video_source}")
            else:
                self.get_logger().info(f"✅ USB Kamera yayını başladı (ID: {video_source})")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Yüksek çözünürlüklü telefon videoları sistemi inanılmaz yavaşlatır!
            # Yayınlamadan önce her zaman 640x480'e küçült.
            frame = cv2.resize(frame, (640, 480))
            
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(msg)
        else:
            if self.is_file:
                # Video bittiyse başa sar (loop özelliği)
                self.get_logger().info("Video bitti, başa sarılıyor...")
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            else:
                self.get_logger().warn("Kameradan kare okunamadi, baglanti bekleniyor...")

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release() # Kapanırken kamerayı serbest bırak
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()