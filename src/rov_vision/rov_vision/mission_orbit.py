#!/usr/bin/env python3
"""
=============================================================================
MISSION ORBIT — Şamandıra Yörünge Görev Kontrolcüsü (OpenCV + PID)
=============================================================================
Teknofest İnsansız Su Altı Sistemleri Yarışması — Antigravity Takımı

── MOTOR KONTROLÜ ──
autonomous_driver.py ile BİREBİR AYNI Serial PWM protokolü kullanılır:
  - Serial port : COM8 (parametre ile değiştirilebilir)
  - Baud rate   : 115200
  - Paket formatı: "x1,y1,x2,y2\n"
  - PWM aralığı : 1060-1940 (nötr: 1500)
  - Kanal eşlemeleri:
      x1 → Dönüş   (angular.z)
      y1 → İleri    (linear.x)
      x2 → Yanaşma  (linear.y)
      y2 → Derinlik (linear.z)

Görev Akışı:
  INIT → GOTO_WAYPOINT → VISUAL_SEARCH → APPROACH → ORBITING → GOTO_CENTER → DONE

Özellikler:
  • Kontur tabanlı gelişmiş kırmızı şamandıra tespiti (çift HSV aralığı)
  • PID tabanlı Visual Servoing (şamandırayı kamera merkezine kilitleme)
  • Orbit sırasında mesafe + yön PID ile kararlı çember yörüngesi
  • Tüm parametreler üst kısımda kolayca kalibre edilebilir
  • Doğrudan Serial PWM ile Arduino motor kontrolü

Yayınlar:
  /cmd_vel (geometry_msgs/Twist)  — Debug/rosbag kaydı için
  /orbit/debug_image (sensor_msgs/Image)  — OpenCV debug görüntüsü

Abonelikler:
  /camera/image_raw (sensor_msgs/Image)  — Ön kamera görüntüsü
=============================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import serial
import time
import math


# ══════════════════════════════════════════════════════════════════
#  YARDIMCI FONKSİYONLAR (autonomous_driver.py ile birebir aynı)
# ══════════════════════════════════════════════════════════════════
def map_value(val, in_min, in_max, out_min, out_max):
    """Manuel sürüş kodundaki map fonksiyonu — değiştirilmedi."""
    return int((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


def clamp(val, lo, hi):
    """Değeri sınırlar içinde tutar."""
    return max(lo, min(hi, val))


# ══════════════════════════════════════════════════════════════════
#  ORBIT MISSION NODE
# ══════════════════════════════════════════════════════════════════
class OrbitMissionNode(Node):
    """
    Şamandıra yörünge görevini yöneten state machine tabanlı ROS 2 node'u.

    Dead Reckoning ile waypoint'e gider, OpenCV ile şamandırayı bulur,
    PID ile kilitlenir, yaklaşır ve şamandıra etrafında yörünge çizer.
    Sonunda merkez bölgeye park eder.

    Motor komutları Serial PWM üzerinden doğrudan Arduino'ya gönderilir.
    """

    # ╔═══════════════════════════════════════════════════════════════════════╗
    # ║                    KALİBRASYON PARAMETRELERİ                        ║
    # ╚═══════════════════════════════════════════════════════════════════════╝

    # ── PWM sabitleri (autonomous_driver.py ile birebir aynı) ────────────────
    PWM_MIN     = 1060
    PWM_MAX     = 1940
    PWM_NEUTRAL = 1500

    # ── HSV Renk Eşikleri (Kırmızı şamandıra) ───────────────────────────────
    HSV_LOWER_1 = np.array([0,   120,  70])
    HSV_UPPER_1 = np.array([10,  255, 255])
    HSV_LOWER_2 = np.array([170, 120,  70])
    HSV_UPPER_2 = np.array([180, 255, 255])

    # ── Morfoloji / Gürültü Filtreleme ───────────────────────────────────────
    MORPH_KERNEL_SIZE = 5
    MIN_CONTOUR_AREA  = 800

    # ── PID Katsayıları — Yatay Kilitleme (Visual Servoing) ──────────────────
    YAW_KP = 0.8
    YAW_KI = 0.02
    YAW_KD = 0.15

    # ── PID Katsayıları — Mesafe Kontrolü (Orbit sırasında) ──────────────────
    DIST_KP = 0.5
    DIST_KI = 0.01
    DIST_KD = 0.10

    # ── PID Limitleri ────────────────────────────────────────────────────────
    PID_INTEGRAL_LIMIT = 0.5
    MAX_ANGULAR_Z      = 0.6
    MAX_LINEAR_X       = 0.5
    MAX_LINEAR_Y       = 0.5

    # ── Durum Süreleri (Dead Reckoning) ──────────────────────────────────────
    INIT_WAIT_DURATION     = 5.0
    BLIND_DRIVE_DURATION   = 16.0
    BLIND_DRIVE_SPEED      = 0.5
    SEARCH_YAW_SPEED       = 0.25
    SEARCH_TIMEOUT         = 30.0
    APPROACH_SPEED         = 0.3
    ORBIT_DURATION         = 25.0
    ORBIT_LATERAL_SPEED    = 0.35
    GOTO_CENTER_SPEED      = 0.5
    GOTO_CENTER_DURATION   = 6.0

    # ── Yaklaşma / Orbit Hedef Değerleri ─────────────────────────────────────
    TARGET_AREA_RATIO    = 0.08
    APPROACH_AREA_TOLERANCE = 0.02
    LOCK_ERROR_THRESHOLD    = 0.05

    # ── Kamera Ayarları ──────────────────────────────────────────────────────
    CAMERA_TOPIC = '/camera/image_raw'

    # ╔═══════════════════════════════════════════════════════════════════════╗
    # ║                         NODE BAŞLATMA                               ║
    # ╚═══════════════════════════════════════════════════════════════════════╝

    def __init__(self):
        super().__init__('orbit_mission_node')

        # ── Serial port parametreleri ─────────────────────────────────────────
        self.declare_parameter('serial_port', 'COM8')
        self.declare_parameter('baud_rate',   115200)

        # ── Hız limitleri (PWM dönüşümü için) ─────────────────────────────────
        self.declare_parameter('max_linear',  0.5)
        self.declare_parameter('max_angular', 0.8)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate   = self.get_parameter('baud_rate').value
        self.max_linear  = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value

        # ── Serial Port Aç ────────────────────────────────────────────────────
        self.ser = None
        self._open_serial()

        # ── Görev Durumları ──────────────────────────────────────────────────
        self.STATE_INIT           = "INIT"
        self.STATE_GOTO_WAYPOINT  = "GOTO_WAYPOINT"
        self.STATE_VISUAL_SEARCH  = "VISUAL_SEARCH"
        self.STATE_APPROACH       = "APPROACH"
        self.STATE_ORBITING       = "ORBITING"
        self.STATE_GOTO_CENTER    = "GOTO_CENTER"
        self.STATE_DONE           = "DONE"

        self.current_state = self.STATE_INIT

        # ── Zamanlayıcılar ───────────────────────────────────────────────────
        self.state_start_time = time.time()

        # ── OpenCV Tespit Sonuçları ──────────────────────────────────────────
        self.target_detected  = False
        self.target_cx        = 0.0
        self.target_cy        = 0.0
        self.target_area      = 0.0
        self.image_width      = 640
        self.image_height     = 480

        # ── PID İç Durumları — Yaw ───────────────────────────────────────────
        self.yaw_error_integral  = 0.0
        self.yaw_error_prev      = 0.0
        self.yaw_last_time       = time.time()

        # ── PID İç Durumları — Mesafe ────────────────────────────────────────
        self.dist_error_integral = 0.0
        self.dist_error_prev     = 0.0
        self.dist_last_time      = time.time()

        # ── Orbit Açı Takibi ─────────────────────────────────────────────────
        self.orbit_accumulated_yaw = 0.0

        # ── ROS 2 Yayıncı ve Aboneler ───────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/orbit/debug_image', 1)

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            self.CAMERA_TOPIC,
            self.image_callback,
            10
        )

        # Ana döngü — 20 Hz
        self.timer = self.create_timer(0.05, self.mission_loop)

        self.get_logger().info(
            "🚀 Yörünge Görev Kontrolcüsü (Serial PWM + OpenCV + PID) Başladı!\n"
            f"   Serial port  : {self.serial_port} @ {self.baud_rate}\n"
            f"   PWM aralığı  : {self.PWM_MIN}-{self.PWM_MAX} (nötr: {self.PWM_NEUTRAL})\n"
            f"   HSV Aralık 1 : {self.HSV_LOWER_1} — {self.HSV_UPPER_1}\n"
            f"   HSV Aralık 2 : {self.HSV_LOWER_2} — {self.HSV_UPPER_2}\n"
            f"   Min Kontur   : {self.MIN_CONTOUR_AREA} px²\n"
            f"   Yaw PID      : Kp={self.YAW_KP}, Ki={self.YAW_KI}, Kd={self.YAW_KD}\n"
            f"   Dist PID     : Kp={self.DIST_KP}, Ki={self.DIST_KI}, Kd={self.DIST_KD}\n"
            f"   Orbit Süresi : {self.ORBIT_DURATION}s | Yanal Hız: {self.ORBIT_LATERAL_SPEED} m/s"
        )

    # ╔═══════════════════════════════════════════════════════════════════════╗
    # ║            SERIAL PORT (autonomous_driver.py ile aynı)              ║
    # ╚═══════════════════════════════════════════════════════════════════════╝

    def _open_serial(self):
        """Serial portu açar — autonomous_driver.py ile aynı parametreler."""
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f'✅ Serial bağlantı kuruldu: {self.serial_port}')
        except Exception as e:
            self.get_logger().error(
                f'❌ Serial bağlantı BAŞARISIZ: {e}\n'
                f'  → Araç hareket ETMEYECEK! Portu kontrol et.'
            )
            self.ser = None

    def _send_serial(self, x1, y1, x2, y2):
        """
        autonomous_driver.py ile BİREBİR AYNI paket formatı.
        Paket: "{x1},{y1},{x2},{y2}\n"
        """
        x1 = clamp(x1, self.PWM_MIN, self.PWM_MAX)
        y1 = clamp(y1, self.PWM_MIN, self.PWM_MAX)
        x2 = clamp(x2, self.PWM_MIN, self.PWM_MAX)
        y2 = clamp(y2, self.PWM_MIN, self.PWM_MAX)

        paket = f"{x1},{y1},{x2},{y2}\n"

        if self.ser and self.ser.is_open:
            try:
                self.ser.write(paket.encode('utf-8'))
            except Exception as e:
                self.get_logger().warn(f'Serial yazma hatası: {e}')

    # ╔═══════════════════════════════════════════════════════════════════════╗
    # ║       TWIST → PWM DÖNÜŞÜMÜ (autonomous_driver.py ile aynı)         ║
    # ╚═══════════════════════════════════════════════════════════════════════╝

    def _twist_to_pwm(self, twist):
        """
        autonomous_driver.py'deki _twist_to_pwm ile AYNI mantık:
          angular.z → x1 (dönüş)   : pozitif=sola → yüksek PWM=sağa
          linear.x  → y1 (ileri)   : pozitif=ileri → düşük PWM (joystick tersi)
          linear.y  → x2 (yanaşma) : pozitif=sağ  → yüksek PWM
          linear.z  → y2 (derinlik): pozitif=yukarı → yüksek PWM
        """
        # Angular.z → x1 (dönüş)
        ang_z = clamp(twist.angular.z, -self.max_angular, self.max_angular)
        x1 = map_value(ang_z, -self.max_angular, self.max_angular,
                        self.PWM_MAX, self.PWM_MIN)

        # Linear.x → y1 (ileri/geri) — joystick Y ekseni ters!
        lin_x = clamp(twist.linear.x, -self.max_linear, self.max_linear)
        y1 = map_value(lin_x, -self.max_linear, self.max_linear,
                        self.PWM_MAX, self.PWM_MIN)

        # Linear.y → x2 (yanaşma/strafe)
        lin_y = clamp(twist.linear.y, -self.max_linear, self.max_linear)
        x2 = map_value(lin_y, -self.max_linear, self.max_linear,
                        self.PWM_MIN, self.PWM_MAX)

        # Linear.z → y2 (derinlik)
        lin_z = clamp(twist.linear.z, -self.max_linear, self.max_linear)
        y2 = map_value(lin_z, -self.max_linear, self.max_linear,
                        self.PWM_MIN, self.PWM_MAX)

        return x1, y1, x2, y2

    # ╔═══════════════════════════════════════════════════════════════════════╗
    # ║         KOMUT GÖNDER (Twist → Serial PWM + /cmd_vel yayını)        ║
    # ╚═══════════════════════════════════════════════════════════════════════╝

    def _send_command(self, twist):
        """Twist mesajını hem Serial PWM olarak Arduino'ya hem de /cmd_vel'e gönderir."""
        # 1) Serial PWM gönder (ASIL MOTOR KONTROLÜ)
        x1, y1, x2, y2 = self._twist_to_pwm(twist)
        self._send_serial(x1, y1, x2, y2)

        # 2) /cmd_vel yayınla (debug/rosbag kaydı için)
        self.cmd_pub.publish(twist)

    # ╔═══════════════════════════════════════════════════════════════════════╗
    # ║                       DURUM YÖNETİMİ                               ║
    # ╚═══════════════════════════════════════════════════════════════════════╝

    def change_state(self, new_state):
        """Durumu değiştirir, PID değerlerini sıfırlar, kronometreyi başlatır."""
        self.get_logger().info(
            f"🔄 DURUM DEĞİŞTİ: {self.current_state} → {new_state}"
        )
        self.current_state = new_state
        self.state_start_time = time.time()

        # PID birikimlerini sıfırla
        self._reset_yaw_pid()
        self._reset_dist_pid()

        # Orbit sayacını sıfırla
        if new_state == self.STATE_ORBITING:
            self.orbit_accumulated_yaw = 0.0

    # ╔═══════════════════════════════════════════════════════════════════════╗
    # ║                    PID KONTROL FONKSİYONLARI                        ║
    # ╚═══════════════════════════════════════════════════════════════════════╝

    def _reset_yaw_pid(self):
        self.yaw_error_integral = 0.0
        self.yaw_error_prev = 0.0
        self.yaw_last_time = time.time()

    def _reset_dist_pid(self):
        self.dist_error_integral = 0.0
        self.dist_error_prev = 0.0
        self.dist_last_time = time.time()

    def _compute_pid(self, error, kp, ki, kd,
                     integral_ref, prev_error_ref, last_time_ref):
        """Genel amaçlı PID hesaplayıcısı."""
        now = time.time()
        dt = now - last_time_ref
        if dt <= 0.0 or dt > 1.0:
            dt = 0.05

        p_term = kp * error

        new_integral = integral_ref + error * dt
        new_integral = max(-self.PID_INTEGRAL_LIMIT,
                           min(self.PID_INTEGRAL_LIMIT, new_integral))
        i_term = ki * new_integral

        d_term = kd * (error - prev_error_ref) / dt

        output = p_term + i_term + d_term
        return output, new_integral, error, now

    def compute_yaw_pid(self, error):
        """Yaw PID hesaplar. Çıkış: angular_z komutu."""
        output, self.yaw_error_integral, self.yaw_error_prev, self.yaw_last_time = \
            self._compute_pid(
                error,
                self.YAW_KP, self.YAW_KI, self.YAW_KD,
                self.yaw_error_integral,
                self.yaw_error_prev,
                self.yaw_last_time
            )
        return max(-self.MAX_ANGULAR_Z, min(self.MAX_ANGULAR_Z, output))

    def compute_dist_pid(self, error):
        """Mesafe PID hesaplar. Çıkış: linear_x komutu."""
        output, self.dist_error_integral, self.dist_error_prev, self.dist_last_time = \
            self._compute_pid(
                error,
                self.DIST_KP, self.DIST_KI, self.DIST_KD,
                self.dist_error_integral,
                self.dist_error_prev,
                self.dist_last_time
            )
        return max(-self.MAX_LINEAR_X, min(self.MAX_LINEAR_X, output))

    # ╔═══════════════════════════════════════════════════════════════════════╗
    # ║                  OPENCV GÖRÜNTÜ İŞLEME (GÖZLER)                    ║
    # ╚═══════════════════════════════════════════════════════════════════════╝

    def image_callback(self, msg):
        """
        Kamera görüntüsünü işler, kırmızı şamandırayı kontur analizi ile tespit eder.

        Sadece VISUAL_SEARCH, APPROACH ve ORBITING durumlarında aktiftir.
        """
        active_states = (
            self.STATE_VISUAL_SEARCH,
            self.STATE_APPROACH,
            self.STATE_ORBITING
        )
        if self.current_state not in active_states:
            self.target_detected = False
            return

        try:
            # ── 1. ROS Image → OpenCV BGR ────────────────────────────────────
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_height, self.image_width = frame.shape[:2]

            # ── 2. BGR → HSV Dönüşümü ───────────────────────────────────────
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # ── 3. Çift Aralık Kırmızı Maskesi ──────────────────────────────
            mask1 = cv2.inRange(hsv, self.HSV_LOWER_1, self.HSV_UPPER_1)
            mask2 = cv2.inRange(hsv, self.HSV_LOWER_2, self.HSV_UPPER_2)
            mask = cv2.bitwise_or(mask1, mask2)

            # ── 4. Morfolojik Gürültü Temizleme ─────────────────────────────
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (self.MORPH_KERNEL_SIZE, self.MORPH_KERNEL_SIZE)
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

            # ── 5. Kontur Bulma ──────────────────────────────────────────────
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            # ── 6. En Büyük Konturu Seç ve Filtrele ─────────────────────────
            debug_frame = frame.copy()
            self.target_detected = False

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)

                if area >= self.MIN_CONTOUR_AREA:
                    # ── 7. Moments ile Merkez Hesaplama ──────────────────────
                    M = cv2.moments(largest_contour)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        self.target_detected = True
                        self.target_cx   = float(cx)
                        self.target_cy   = float(cy)
                        self.target_area = float(area)

                        # ── Debug Çizimi ─────────────────────────────────────
                        cv2.drawContours(debug_frame, [largest_contour], -1,
                                         (0, 255, 0), 2)
                        cv2.circle(debug_frame, (cx, cy), 8, (0, 0, 255), -1)
                        center_x = self.image_width // 2
                        cv2.line(debug_frame, (center_x, 0),
                                 (center_x, self.image_height),
                                 (255, 255, 0), 1)
                        cv2.line(debug_frame, (cx, cy),
                                 (center_x, cy), (255, 0, 0), 2)

                        area_ratio = area / (self.image_width * self.image_height)
                        norm_err = (cx - center_x) / (self.image_width / 2)
                        cv2.putText(
                            debug_frame,
                            f"CX:{cx} CY:{cy} Alan:{area:.0f} "
                            f"Oran:{area_ratio:.3f} Hata:{norm_err:.2f}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (0, 255, 255), 2
                        )

            # Durum bilgisi
            cv2.putText(
                debug_frame,
                f"Durum: {self.current_state} | "
                f"Hedef: {'EVET' if self.target_detected else 'HAYIR'}",
                (10, self.image_height - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (0, 255, 0) if self.target_detected else (0, 0, 255), 2
            )

            # Debug görüntüsünü yayınla
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            except Exception:
                pass

        except Exception as e:
            self.get_logger().error(f"Kamera işleme hatası: {e}")

    # ╔═══════════════════════════════════════════════════════════════════════╗
    # ║                  GÖREV DÖNGÜSÜ (BEYİN / STATE MACHINE)             ║
    # ╚═══════════════════════════════════════════════════════════════════════╝

    def mission_loop(self):
        """
        20 Hz ana karar döngüsü. Her çağrıda mevcut duruma göre
        Twist komutu üretir, Serial PWM olarak Arduino'ya gönderir
        ve /cmd_vel'e yayınlar.
        """
        twist = Twist()
        elapsed = time.time() - self.state_start_time

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # DURUM: INIT — Başlatma ve Sensör Stabilizasyonu
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        if self.current_state == self.STATE_INIT:
            if elapsed > self.INIT_WAIT_DURATION:
                self.change_state(self.STATE_GOTO_WAYPOINT)

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # DURUM: GOTO_WAYPOINT — Kör İleri Sürüş (Dead Reckoning)
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        elif self.current_state == self.STATE_GOTO_WAYPOINT:
            twist.linear.x = self.BLIND_DRIVE_SPEED

            if elapsed > self.BLIND_DRIVE_DURATION:
                self.get_logger().info(
                    f"📍 Waypoint'e ulaşıldı (tahmini). "
                    f"Süre: {elapsed:.1f}s. Görsel aramaya geçiliyor."
                )
                self.change_state(self.STATE_VISUAL_SEARCH)

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # DURUM: VISUAL_SEARCH — Şamandıra Arama + PID Kilitleme
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        elif self.current_state == self.STATE_VISUAL_SEARCH:
            if self.target_detected:
                center_x = self.image_width / 2.0
                error = (self.target_cx - center_x) / (self.image_width / 2.0)

                angular_z = self.compute_yaw_pid(error)

                if abs(error) < self.LOCK_ERROR_THRESHOLD:
                    self.get_logger().info(
                        f"🎯 HEDEF KİLİTLENDİ! Hata: {error:.3f}. "
                        f"Yaklaşma başlatılıyor."
                    )
                    self.change_state(self.STATE_APPROACH)
                else:
                    twist.angular.z = -angular_z
                    self.get_logger().info(
                        f"🔍 Arama: Hedef görüldü | "
                        f"Hata: {error:.3f} | "
                        f"Dönüş: {twist.angular.z:.3f} rad/s",
                        throttle_duration_sec=0.5
                    )
            else:
                twist.angular.z = self.SEARCH_YAW_SPEED

                if elapsed > self.SEARCH_TIMEOUT:
                    self.get_logger().warn(
                        f"⚠️ Arama zaman aşımı ({self.SEARCH_TIMEOUT}s)! "
                        f"Yörünge atlanıyor, merkeze yöneliniyor."
                    )
                    self.change_state(self.STATE_GOTO_CENTER)

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # DURUM: APPROACH — Şamandıraya Yaklaşma (PID Mesafe Kontrolü)
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        elif self.current_state == self.STATE_APPROACH:
            if self.target_detected:
                # Yaw PID
                center_x = self.image_width / 2.0
                yaw_error = (self.target_cx - center_x) / (self.image_width / 2.0)
                angular_z = self.compute_yaw_pid(yaw_error)
                twist.angular.z = -angular_z

                # Mesafe kontrolü
                total_pixels = self.image_width * self.image_height
                current_ratio = self.target_area / total_pixels
                dist_error = (self.TARGET_AREA_RATIO - current_ratio) / self.TARGET_AREA_RATIO

                if dist_error > self.APPROACH_AREA_TOLERANCE:
                    forward_speed = self.compute_dist_pid(dist_error)
                    twist.linear.x = max(0.0, forward_speed)
                    self.get_logger().info(
                        f"🏊 Yaklaşma: Alan oranı={current_ratio:.4f} "
                        f"Hedef={self.TARGET_AREA_RATIO:.4f} "
                        f"Hız={twist.linear.x:.3f} m/s",
                        throttle_duration_sec=0.5
                    )
                else:
                    self.get_logger().info(
                        f"✅ Yaklaşma tamamlandı! Alan oranı: {current_ratio:.4f}. "
                        f"Yörünge başlatılıyor."
                    )
                    self.change_state(self.STATE_ORBITING)
            else:
                self.get_logger().warn(
                    "⚠️ Yaklaşma sırasında hedef kayboldu! Aramaya dönülüyor."
                )
                self.change_state(self.STATE_VISUAL_SEARCH)

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # DURUM: ORBITING — Yörünge Turu (Yengeç Hareketi + PID)
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        elif self.current_state == self.STATE_ORBITING:
            if self.target_detected:
                # Yaw PID — Şamandırayı merkezde tut
                center_x = self.image_width / 2.0
                yaw_error = (self.target_cx - center_x) / (self.image_width / 2.0)
                angular_z = self.compute_yaw_pid(yaw_error)
                twist.angular.z = -angular_z

                # Mesafe PID — Sabit mesafeyi koru
                total_pixels = self.image_width * self.image_height
                current_ratio = self.target_area / total_pixels
                dist_error = (self.TARGET_AREA_RATIO - current_ratio) / self.TARGET_AREA_RATIO
                forward_correction = self.compute_dist_pid(dist_error)
                twist.linear.x = forward_correction

                # Yanal hız — Yengeç hareketi
                twist.linear.y = self.ORBIT_LATERAL_SPEED

                self.get_logger().info(
                    f"🔄 Yörünge: {elapsed:.1f}/{self.ORBIT_DURATION:.0f}s | "
                    f"Yaw hata: {yaw_error:.3f} | "
                    f"Alan oranı: {current_ratio:.4f} | "
                    f"İleri düzeltme: {twist.linear.x:.3f}",
                    throttle_duration_sec=1.0
                )

            else:
                twist.linear.y = self.ORBIT_LATERAL_SPEED
                twist.angular.z = self.SEARCH_YAW_SPEED * 0.5
                self.get_logger().warn(
                    "⚠️ Yörünge sırasında hedef geçici kayıp! "
                    "Kör yörünge devam ediyor.",
                    throttle_duration_sec=1.0
                )

            if elapsed > self.ORBIT_DURATION:
                self.get_logger().info(
                    f"🏁 Yörünge turu tamamlandı! "
                    f"Süre: {elapsed:.1f}s. Merkeze yöneliniyor."
                )
                self.change_state(self.STATE_GOTO_CENTER)

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # DURUM: GOTO_CENTER — Merkez Bölgeye Park Sürüşü
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        elif self.current_state == self.STATE_GOTO_CENTER:
            twist.linear.x = self.GOTO_CENTER_SPEED

            if elapsed > self.GOTO_CENTER_DURATION:
                self.get_logger().info(
                    "🅿️ Merkeze park tamamlandı! Görev sona erdi."
                )
                self.change_state(self.STATE_DONE)

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # DURUM: DONE — Görev Tamamlandı, Motorlar Nötr
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        elif self.current_state == self.STATE_DONE:
            if elapsed < 1.0:
                self.get_logger().info(
                    "✅ GÖREV TAMAMLANDI — Motorlar nötr.",
                    throttle_duration_sec=5.0
                )

        # ── Komutu gönder (Serial PWM + /cmd_vel) ────────────────────────────
        self._send_command(twist)


# =============================================================================
# MAIN
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = OrbitMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Kullanıcı tarafından durduruldu.')
    finally:
        # Güvenli kapanış: motorları nötre al
        stop_twist = Twist()
        x1, y1, x2, y2 = node._twist_to_pwm(stop_twist)
        node._send_serial(x1, y1, x2, y2)
        node.cmd_pub.publish(stop_twist)
        # Serial portu kapat
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()