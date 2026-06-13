import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        # Görüntüyü yayınlayacağımız ROS 2 kanalı (topic)
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Saniyede 20 kare (20 FPS) için 0.05 saniyelik zamanlayıcı
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.bridge = CvBridge()
        
        # 1. Linux video sürücüsünü (V4L2) kullanmaya zorluyoruz
        self.cap = cv2.VideoCapture(0)
        
        # 2. WSL darboğazını aşmak için donanımsal MJPG sıkıştırması açıyoruz
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        
        # 3. Çözünürlük ve FPS ayarları (Çizgi takibi için ideal değerler)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 20)
        
        # 4. Gecikmeyi (latency) önlemek için tampon belleği küçültüyoruz
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        if not self.cap.isOpened():
            self.get_logger().error("USB Kamera bulunamadi! /dev/video0 izinlerini kontrol et.")
        else:
            self.get_logger().info("✅ USB Kamera canli yayini (MJPG/V4L2) basladi!")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # OpenCV (BGR) görüntüsünü ROS 2 (sensor_msgs/Image) formatına çevir
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(msg)
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