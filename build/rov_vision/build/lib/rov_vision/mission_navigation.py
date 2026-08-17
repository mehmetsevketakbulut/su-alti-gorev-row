#!/usr/bin/env python3
"""
=============================================================================
MISSION NAVIGATION — Otonom İntikal ve Alan Geçişi (Görev 2)
=============================================================================
Teknofest İnsansız Su Altı Sistemleri Yarışması — Antigravity Takımı

Görev 2 Kuralları:
  1. Su altında otonom navigasyon (Kör sürüş / Dead Reckoning).
  2. Başlangıç -> Dönüş Şamandırası -> Bitiş Alanı koordinatları.
  3. Şamandıra etrafında tur atma.
  4. Bitiş (karesel) alanı bulup içinde yüzeye çıkma.

Motor kontrolü autonomous_driver.py'deki gibi Serial PWM kullanır.

Matematik ve Navigasyon Mantığı:
  - GPS çekmediği için robot su üstünde 3 koordinatı alır.
  - Haversine formülü ile mesafeler ve Pusula Açıları (Bearing) hesaplanır.
  - BMI085 IMU'dan gelen Gyro Z (angular_velocity.z) integre edilerek 
    göreceli Heading (Yaw) bulunur.
  - Zaman bazlı kör sürüş (Dead Reckoning) IMU PID ile düzeltilir.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import serial
import time
import math

# ══════════════════════════════════════════════════════════════════
#  GPS MATEMATİĞİ (Haversine & Bearing)
# ══════════════════════════════════════════════════════════════════
def haversine_distance(lat1, lon1, lat2, lon2):
    """İki GPS koordinatı arasındaki mesafeyi (metre) hesaplar."""
    R = 6371000  # Dünya yarıçapı (metre)
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2.0)**2 + \
        math.cos(phi1) * math.cos(phi2) * \
        math.sin(delta_lambda / 2.0)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def calculate_bearing(lat1, lon1, lat2, lon2):
    """Başlangıç noktasından hedefe olan pusula açısını (0-360 derece) hesaplar."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_lambda = math.radians(lon2 - lon1)

    y = math.sin(delta_lambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - \
        math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
    
    bearing = math.atan2(y, x)
    return (math.degrees(bearing) + 360) % 360

# ══════════════════════════════════════════════════════════════════
#  YARDIMCI FONKSİYONLAR
# ══════════════════════════════════════════════════════════════════
def map_value(val, in_min, in_max, out_min, out_max):
    return int((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

# ══════════════════════════════════════════════════════════════════
#  ANA NODE
# ══════════════════════════════════════════════════════════════════
class MissionNavigationNode(Node):
    def __init__(self):
        super().__init__('mission_navigation_node')

        # ── Görev Koordinatları (Parametre Olarak Alınır) ─────────────
        # Varsayılan değerleri yarışmada değiştireceksiniz
        self.declare_parameter('start_lat', 41.000000)
        self.declare_parameter('start_lon', 29.000000)
        
        self.declare_parameter('buoy_lat', 41.000100) # Şamandıra
        self.declare_parameter('buoy_lon', 29.000000)
        
        self.declare_parameter('end_lat', 41.000200)  # Bitiş (Karesel Alan)
        self.declare_parameter('end_lon', 29.000000)

        self.start_lat = self.get_parameter('start_lat').value
        self.start_lon = self.get_parameter('start_lon').value
        self.buoy_lat = self.get_parameter('buoy_lat').value
        self.buoy_lon = self.get_parameter('buoy_lon').value
        self.end_lat = self.get_parameter('end_lat').value
        self.end_lon = self.get_parameter('end_lon').value

        # ── Serial Port ────────────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyUSB0') # Jetson için genelde böyledir
        self.declare_parameter('baud_rate', 115200)
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value

        # ── PWM Sınırları ──────────────────────────────────────────────
        self.PWM_MIN = 1060
        self.PWM_MAX = 1940
        self.PWM_NEUTRAL = 1500

        # ── Görev Hızları ve Parametreler ──────────────────────────────
        self.CRUISE_SPEED = 0.5        # İleri gidiş hızı (normalize)
        self.CRUISE_METERS_PER_SEC = 0.5 # Aracın 0.5 hızda saniyede gittiği metre (Test edilmeli!)
        self.TARGET_DEPTH_CM = 100.0   # 1 metre derinden gidecek
        
        self.ORBIT_LATERAL_SPEED = 0.35
        self.ORBIT_DURATION = 25.0

        # ── PID Katsayıları ────────────────────────────────────────────
        self.YAW_KP = 0.02   # Pusula dönüş düzeltmesi (derece üzerinden)
        self.YAW_KD = 0.005
        
        self.DEPTH_KP = 0.005 # Derinlik sabitleme
        
        self.VISUAL_YAW_KP = 0.8  # Kamera ile şamandıra takibi
        self.VISUAL_YAW_KD = 0.15

        # ── Kamera ve Renk Eşikleri ────────────────────────────────────
        self.HSV_LOWER = np.array([0, 0, 0])
        self.HSV_UPPER = np.array([180, 255, 45]) # Siyah / Koyu renk için varsayılan
        self.MIN_CONTOUR_AREA = 800

        # ── İç Durumlar (Sensörler) ────────────────────────────────────
        self.current_yaw_deg = 0.0 # Gyro'dan hesaplanan Göreceli Pusula
        self.last_imu_time = time.time()
        
        self.current_depth_cm = 0.0

        self.target_detected = False
        self.target_cx = 0.0
        self.target_area = 0.0
        self.image_width = 640

        # ── Durum Makinesi (State Machine) ─────────────────────────────
        self.STATE_CALCULATE    = "CALCULATE"
        self.STATE_DIVE         = "DIVE"
        self.STATE_CRUISE_BUOY  = "CRUISE_BUOY"
        self.STATE_SEARCH_BUOY  = "SEARCH_BUOY"
        self.STATE_ORBIT        = "ORBIT"
        self.STATE_TURN_END     = "TURN_END"
        self.STATE_CRUISE_END   = "CRUISE_END"
        self.STATE_SEARCH_END   = "SEARCH_END"
        self.STATE_SURFACE      = "SURFACE"
        self.STATE_DONE         = "DONE"

        self.state = self.STATE_CALCULATE
        self.state_start_time = time.time()

        # ── Hesaplanan Navigasyon Verileri ─────────────────────────────
        self.dist_to_buoy = 0.0
        self.bearing_to_buoy = 0.0
        self.time_to_buoy = 0.0
        
        self.dist_to_end = 0.0
        self.bearing_to_end = 0.0
        self.time_to_end = 0.0
        
        self.target_yaw = 0.0 # O an gitmek istediğimiz yön

        # ── Serial Açılışı ─────────────────────────────────────────────
        self.ser = None
        self._open_serial()

        # ── ROS2 Tanımlamaları ─────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(Float32, '/depth_sensor', self.depth_callback, 10)

        self.timer = self.create_timer(0.05, self.mission_loop) # 20 Hz
        
        self.get_logger().info("🚀 Görev 2 Navigasyon Sistemi Başlatıldı! Hesaplamalar yapılıyor...")

    # ══════════════════════════════════════════════════════════════════
    #  SENSÖR CALLBACK'LERİ
    # ══════════════════════════════════════════════════════════════════
    def imu_callback(self, msg: Imu):
        """
        Göreceli Heading (Yaw) Hesaplama.
        Cihazda manyetometre (pusula) olmadığı için Z ekseni jiroskopunu 
        (angular_velocity.z) zamana göre integre ediyoruz.
        """
        now = time.time()
        dt = now - self.last_imu_time
        if dt > 0 and dt < 0.2:
            # rad/s to deg/s
            gz_deg = math.degrees(msg.angular_velocity.z)
            # ROS'ta genelde Z ekseninde pozitif sola dönüştür. (Sağ el kuralı)
            self.current_yaw_deg += gz_deg * dt
        self.last_imu_time = now

    def depth_callback(self, msg: Float32):
        self.current_depth_cm = msg.data

    def image_callback(self, msg: Image):
        active_states = (self.STATE_SEARCH_BUOY, self.STATE_ORBIT, self.STATE_SEARCH_END)
        if self.state not in active_states:
            self.target_detected = False
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_width = frame.shape[1]
            
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.HSV_LOWER, self.HSV_UPPER)
            
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Şekil filtresi: Sadece dairesel/oval hatlara sahip konturları al
            valid_contours = []
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area >= self.MIN_CONTOUR_AREA:
                    perimeter = cv2.arcLength(cnt, True)
                    if perimeter > 0:
                        circularity = 4 * math.pi * (area / (perimeter * perimeter))
                        # 1.0 tam dairedir. 0.4 diyerek biraz bozuk/oval şekillere de izin veriyoruz.
                        # İnce uzun nesneleri (kablo, boru, kaya) filtreler.
                        if circularity > 0.4:
                            valid_contours.append(cnt)

            self.target_detected = False
            if valid_contours:
                # Geçerli şekiller arasından en büyüğünü seç
                largest = max(valid_contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)
                M = cv2.moments(largest)
                if M["m00"] > 0:
                    self.target_cx = int(M["m10"] / M["m00"])
                    self.target_area = float(area)
                    self.target_detected = True
                        
        except Exception as e:
            self.get_logger().error(f"Kamera hatası: {e}")

    # ══════════════════════════════════════════════════════════════════
    #  SERIAL İLETİŞİM VE MOTOR KONTROLÜ
    # ══════════════════════════════════════════════════════════════════
    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info('✅ Serial bağlandı.')
        except Exception as e:
            self.get_logger().error(f'❌ Serial Hatası: {e}')
            self.ser = None

    def _twist_to_pwm_and_send(self, twist):
        # Yanal hareketler
        ang_z = clamp(twist.angular.z, -1.0, 1.0)
        lin_x = clamp(twist.linear.x, -1.0, 1.0)
        lin_y = clamp(twist.linear.y, -1.0, 1.0)
        lin_z = clamp(twist.linear.z, -1.0, 1.0)

        x1 = map_value(ang_z, -1.0, 1.0, self.PWM_MAX, self.PWM_MIN) # Dönüş
        y1 = map_value(lin_x, -1.0, 1.0, self.PWM_MAX, self.PWM_MIN) # İleri
        x2 = map_value(lin_y, -1.0, 1.0, self.PWM_MIN, self.PWM_MAX) # Yengeç
        y2 = map_value(lin_z, -1.0, 1.0, self.PWM_MIN, self.PWM_MAX) # Derinlik

        paket = f"{x1},{y1},{x2},{y2}\n"
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(paket.encode('utf-8'))
            except:
                pass

        self.cmd_pub.publish(twist)

    def change_state(self, new_state):
        self.get_logger().info(f"🔄 DURUM DEĞİŞTİ: {self.state} → {new_state}")
        self.state = new_state
        self.state_start_time = time.time()

    # ══════════════════════════════════════════════════════════════════
    #  GÖREV DÖNGÜSÜ (STATE MACHINE)
    # ══════════════════════════════════════════════════════════════════
    def mission_loop(self):
        twist = Twist()
        elapsed = time.time() - self.state_start_time

        # ── 1. HESAPLAMA ────────────────────────────────────────────────
        if self.state == self.STATE_CALCULATE:
            self.dist_to_buoy = haversine_distance(self.start_lat, self.start_lon, self.buoy_lat, self.buoy_lon)
            self.bearing_to_buoy = calculate_bearing(self.start_lat, self.start_lon, self.buoy_lat, self.buoy_lon)
            
            self.dist_to_end = haversine_distance(self.buoy_lat, self.buoy_lon, self.end_lat, self.end_lon)
            self.bearing_to_end = calculate_bearing(self.buoy_lat, self.buoy_lon, self.end_lat, self.end_lon)
            
            # Kör sürüş süreleri (Mesafe / Saniyedeki Hız)
            self.time_to_buoy = self.dist_to_buoy / self.CRUISE_METERS_PER_SEC
            self.time_to_end = self.dist_to_end / self.CRUISE_METERS_PER_SEC

            self.get_logger().info(f"📍 Start -> Şamandıra: {self.dist_to_buoy:.1f}m, Açısı: {self.bearing_to_buoy:.1f}°")
            self.get_logger().info(f"📍 Şamandıra -> Bitiş: {self.dist_to_end:.1f}m, Açısı: {self.bearing_to_end:.1f}°")
            
            # Cihaz suya konduğunda doğrudan şamandıraya baktığı varsayılır (Göreceli 0 derece)
            self.target_yaw = 0.0 
            self.current_yaw_deg = 0.0
            
            if elapsed > 2.0: # 2 saniye bekle
                self.change_state(self.STATE_DIVE)

        # ── 2. DALMA ────────────────────────────────────────────────────
        elif self.state == self.STATE_DIVE:
            depth_error = self.TARGET_DEPTH_CM - self.current_depth_cm
            dive_speed = depth_error * self.DEPTH_KP
            twist.linear.z = clamp(dive_speed, -1.0, 1.0)
            
            # Yeterince derine indik mi?
            if abs(depth_error) < 15.0 and elapsed > 5.0:
                self.change_state(self.STATE_CRUISE_BUOY)

        # ── 3. ŞAMANDIRAYA KÖR SÜRÜŞ ────────────────────────────────────
        elif self.state == self.STATE_CRUISE_BUOY:
            # İleri git
            twist.linear.x = self.CRUISE_SPEED
            
            # IMU Yaw PID (Akıntıya karşı burnu düz tut)
            yaw_error = self.target_yaw - self.current_yaw_deg
            twist.angular.z = clamp(yaw_error * self.YAW_KP, -0.6, 0.6)
            
            # Derinliği koru
            twist.linear.z = clamp((self.TARGET_DEPTH_CM - self.current_depth_cm) * self.DEPTH_KP, -0.5, 0.5)
            
            if elapsed > self.time_to_buoy * 0.8: # Mesafenin %80'ine geldiğinde kamerayı aç
                self.change_state(self.STATE_SEARCH_BUOY)

        # ── 4. ŞAMANDIRA GÖRSEL ARAMA VE YAKLAŞMA ───────────────────────
        elif self.state == self.STATE_SEARCH_BUOY:
            twist.linear.z = clamp((self.TARGET_DEPTH_CM - self.current_depth_cm) * self.DEPTH_KP, -0.5, 0.5)
            
            if self.target_detected:
                # Merkeze hizala
                err = (self.target_cx - self.image_width/2) / (self.image_width/2)
                twist.angular.z = -clamp(err * self.VISUAL_YAW_KP, -0.6, 0.6)
                twist.linear.x = 0.3 # Yavaşça yaklaş
                
                if self.target_area > 15000: # Yeterince yaklaştık
                    self.change_state(self.STATE_ORBIT)
            else:
                # Hafifçe dönerek ara
                twist.angular.z = 0.2
                twist.linear.x = 0.1
                
                # Çok uzun süre bulamazsa Bitişe yönel (Güvenlik)
                if elapsed > 30.0:
                    self.get_logger().warn("Şamandıra bulunamadı, Bitişe geçiliyor!")
                    self.change_state(self.STATE_TURN_END)

        # ── 5. YÖRÜNGE (ORBIT) ──────────────────────────────────────────
        elif self.state == self.STATE_ORBIT:
            twist.linear.z = clamp((self.TARGET_DEPTH_CM - self.current_depth_cm) * self.DEPTH_KP, -0.5, 0.5)
            
            if self.target_detected:
                err = (self.target_cx - self.image_width/2) / (self.image_width/2)
                twist.angular.z = -clamp(err * self.VISUAL_YAW_KP, -0.6, 0.6)
            else:
                twist.angular.z = 0.2
                
            twist.linear.y = self.ORBIT_LATERAL_SPEED # Yengeç yürüyüşü
            
            if elapsed > self.ORBIT_DURATION:
                self.change_state(self.STATE_TURN_END)

        # ── 6. BİTİŞE DÖNÜŞ (IMU) ───────────────────────────────────────
        elif self.state == self.STATE_TURN_END:
            twist.linear.z = clamp((self.TARGET_DEPTH_CM - self.current_depth_cm) * self.DEPTH_KP, -0.5, 0.5)
            
            # Dönmemiz gereken relatif açı
            turn_angle = self.bearing_to_end - self.bearing_to_buoy
            
            # Açı sınırlandırması (-180, 180 arasına)
            if turn_angle > 180: turn_angle -= 360
            if turn_angle < -180: turn_angle += 360
            
            self.target_yaw = turn_angle # Yeni hedef
            
            yaw_error = self.target_yaw - self.current_yaw_deg
            twist.angular.z = clamp(yaw_error * self.YAW_KP, -0.6, 0.6)
            
            if abs(yaw_error) < 5.0 and elapsed > 3.0: # Hedefe döndük
                self.change_state(self.STATE_CRUISE_END)

        # ── 7. BİTİŞE KÖR SÜRÜŞ ─────────────────────────────────────────
        elif self.state == self.STATE_CRUISE_END:
            twist.linear.x = self.CRUISE_SPEED
            yaw_error = self.target_yaw - self.current_yaw_deg
            twist.angular.z = clamp(yaw_error * self.YAW_KP, -0.6, 0.6)
            twist.linear.z = clamp((self.TARGET_DEPTH_CM - self.current_depth_cm) * self.DEPTH_KP, -0.5, 0.5)
            
            if elapsed > self.time_to_end * 0.8:
                self.change_state(self.STATE_SEARCH_END)

        # ── 8. BİTİŞ ALANI GÖRSEL ARAMA ─────────────────────────────────
        elif self.state == self.STATE_SEARCH_END:
            twist.linear.x = 0.2
            yaw_error = self.target_yaw - self.current_yaw_deg
            twist.angular.z = clamp(yaw_error * self.YAW_KP, -0.4, 0.4)
            twist.linear.z = clamp((self.TARGET_DEPTH_CM - self.current_depth_cm) * self.DEPTH_KP, -0.5, 0.5)
            
            # Eğer kamerada çok büyük bir şekil belirirse alana gelmişizdir
            if self.target_detected and self.target_area > 20000:
                self.change_state(self.STATE_SURFACE)
                
            if elapsed > self.time_to_end * 0.4 + 10.0: # Kör süre çok aşılırsa yüzeye çık
                self.change_state(self.STATE_SURFACE)

        # ── 9. YÜZEYE ÇIKIŞ ─────────────────────────────────────────────
        elif self.state == self.STATE_SURFACE:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            twist.linear.z = -1.0 # Yukarı tam güç
            
            if self.current_depth_cm < 10.0 and elapsed > 5.0: # Su üstüne çıktı
                self.change_state(self.STATE_DONE)

        # ── 10. GÖREV BİTİŞİ ────────────────────────────────────────────
        elif self.state == self.STATE_DONE:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.z = 0.0

        # Motorlara Gönder
        self._twist_to_pwm_and_send(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MissionNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Durduruldu.')
    finally:
        stop = Twist()
        node._twist_to_pwm_and_send(stop)
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
