import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image       # Kamera verisi için
from cv_bridge import CvBridge          # ROS resmini OpenCV resmine çevirmek için
import cv2
import numpy as np
import time

class OrbitMissionNode(Node):
    def __init__(self):
        super().__init__('orbit_mission_node')

        # --- GÖREV DURUMLARI ---
        self.STATE_INIT = "INIT"
        self.STATE_GOTO_WAYPOINT = "GOTO_WAYPOINT"
        self.STATE_VISUAL_SEARCH = "VISUAL_SEARCH"
        self.STATE_ORBITING = "ORBITING"
        self.STATE_GOTO_CENTER = "GOTO_CENTER"
        self.STATE_DONE = "DONE"

        self.current_state = self.STATE_INIT

        # --- ZAMANLAYICILAR VE BAYRAKLAR ---
        self.state_start_time = time.time()
        self.blind_drive_duration = 16.0 
        self.orbit_duration = 10.0       
        self.hedef_goruldu_mu = False    # OpenCV burayı True yapacak!

        # --- YAYINCI VE ABONELER ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Kamera Görüntüsünü Al (Senin sistemindeki topic adıyla değiştir gerekirse)
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            10
        )

        self.timer = self.create_timer(0.05, self.mission_loop)
        self.get_logger().info("🚀 Yörünge Görev Kontrolcüsü (OpenCV Destekli) Başladı!")

    def change_state(self, new_state):
        self.current_state = new_state
        self.state_start_time = time.time()
        self.get_logger().info(f"🔄 DURUM DEĞİŞTİ: {new_state}")

    # ==========================================================
    # GÖZLER BURADA (OPENCV GÖRÜNTÜ İŞLEME)
    # ==========================================================
    def image_callback(self, msg):
        # Eğer araç "Arama" modunda değilse işlemciyi yorma, görüntüyü atla
        if self.current_state != self.STATE_VISUAL_SEARCH:
            return

        try:
            # ROS mesajını OpenCV'nin anlayacağı formata çevir
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Renk tespiti için HSV formatına çevir
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # KIRMIZI şamandıra için renk aralığı (Havuzda kalibre edersiniz)
            lower_red = np.array([0, 120, 70])
            upper_red = np.array([10, 255, 255])
            
            # Sadece kırmızı yerleri beyaz yapan maske oluştur
            mask = cv2.inRange(hsv, lower_red, upper_red)
            
            # Ekranda kaç tane beyaz (kırmızıya ait) piksel var say
            kirmizi_piksel_sayisi = cv2.countNonZero(mask)
            
            # Eğer 500 pikselden fazla kırmızı gördüysek (yani şamandıra yeterince yakınsa)
            if kirmizi_piksel_sayisi > 500:
                self.hedef_goruldu_mu = True
                self.get_logger().info("🎯 HEDEF ŞAMANDIRA GÖRÜLDÜ!")
                
        except Exception as e:
            self.get_logger().error(f"Kamera hatası: {e}")

    # ==========================================================
    # BEYİN BURADA (KARAR MEKANİZMASI)
    # ==========================================================
    def mission_loop(self):
        twist = Twist()
        elapsed_time = time.time() - self.state_start_time

        if self.current_state == self.STATE_INIT:
            if elapsed_time > 5.0:
                self.change_state(self.STATE_GOTO_WAYPOINT)

        elif self.current_state == self.STATE_GOTO_WAYPOINT:
            twist.linear.x = 0.5 
            if elapsed_time > self.blind_drive_duration:
                self.change_state(self.STATE_VISUAL_SEARCH)

        elif self.current_state == self.STATE_VISUAL_SEARCH:
            # Araç şamandırayı görene kadar olduğu yerde yavaşça kendi etrafında dönsün (Tarama)
            twist.angular.z = 0.2
            
            # OpenCV şamandırayı görüp bayrağı kaldırdıysa diğer moda zıpla!
            if self.hedef_goruldu_mu:
                self.hedef_goruldu_mu = False # Sonraki turlar için sıfırla
                self.change_state(self.STATE_ORBITING)

        elif self.current_state == self.STATE_ORBITING:
            twist.linear.y = 0.4  # Sağa Yanaş
            twist.angular.z = 0.3 # Sola Dön (Yörünge)
            if elapsed_time > self.orbit_duration:
                self.change_state(self.STATE_GOTO_CENTER)

        elif self.current_state == self.STATE_GOTO_CENTER:
            twist.linear.x = 0.5
            if elapsed_time > 6.0: 
                self.change_state(self.STATE_DONE)

        elif self.current_state == self.STATE_DONE:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = OrbitMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()