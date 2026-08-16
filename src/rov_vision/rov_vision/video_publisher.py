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
            source_id = int(video_source)
            self.cap = self._find_working_camera(source_id)
        except ValueError:
            self.cap = cv2.VideoCapture(video_source)
            self.is_file = True

        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error(f"❌ Video kaynağı açılamadı: {video_source}")
            self.timer.cancel()
        else:
            if self.is_file:
                self.get_logger().info(f"🎞️ Video dosyasından yayın başladı: {video_source}")
            else:
                self.get_logger().info(f"✅ USB Kamera yayını başladı (ID: {video_source})")

    def _find_working_camera(self, default_id):
        self.get_logger().info("🚀 OpenCV Brute-Force Kamera Tarayıcısı Başlatılıyor...")
        
        # Sadece default_id'yi değil, video0-3 arasını tara
        for i in range(4):
            # 1. Deneme: V4L2 + MJPG
            cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                ret, frame = cap.read()
                if ret and frame is not None and len(frame.shape) == 3:
                    self.get_logger().info(f"✅ KAMERA BULUNDU: /dev/video{i} (V4L2 + MJPG - Kusursuz Renk)")
                    return cap
                cap.release()
                
            # 2. Deneme: V4L2 + YUYV
            cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                ret, frame = cap.read()
                if ret and frame is not None and len(frame.shape) == 3:
                    self.get_logger().info(f"✅ KAMERA BULUNDU: /dev/video{i} (V4L2 + YUYV - Kusursuz Renk)")
                    return cap
                cap.release()
                
            # 3. Deneme: OpenCV Default
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    self.get_logger().warn(f"⚠️ KAMERA BULUNDU: /dev/video{i} (OpenCV Default)")
                    return cap
                cap.release()
                
        return None

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