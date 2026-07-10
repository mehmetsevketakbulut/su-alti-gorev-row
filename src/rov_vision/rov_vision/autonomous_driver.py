#!/usr/bin/env python3
"""
=============================================================================
AUTONOMOUS DRIVER V2 - Yarışma Uyumlu (Hat Takibi & Kapalı Alan İncelemesi)
=============================================================================
Teknofest İnsansız Su Altı Sistemleri Yarışması — Antigravity Takımı

V1'den Değişiklikler:
  1. MESAFE SENSÖRÜ      : Akustik sensör (8-300cm) ile tahtadan mesafe kontrolü
  2. DİKEY PID           : Tahtaya çarpmayı önleyen altitude hold
  3. HAT SONU ALGILAMA   : Çizgi bitince fırıldak yerine tam dur
  4. MISSION_READY       : Mini ROV bırakma için operatöre sinyal
  5. ACİL ÇARPMA KORUMASI: Kritik mesafede tüm motorları kes, geri çekil
  6. EĞİMLİ TAHTA DESTEĞİ: PID otomatik olarak eğime adapte olur

Durum Makinesi (State Machine):
  SEARCHING → FOLLOWING → RECOVERING → END_OF_LINE → MISSION_READY
                                      ↘ SEARCHING (kısa kayıp)
                                      ↘ LOST (uzun kayıp, takip geçmişi yok)
  EMERGENCY (mesafe kritik) → herhangi durumda tetiklenebilir

Serial Protokol (Manuel sürüşle BİREBİR AYNI):
  - Port: COM8 (parametre ile ayarlanır)
  - Baud: 115200
  - Paket: "x1,y1,x2,y2\n"
  - PWM aralığı: 1060-1940 (nötr: 1500)
  - Kanal eşlemeleri:
      x1 → Dönüş     (yaw)    → çizgi takip angular.z
      y1 → İleri      (surge)  → çizgi takip linear.x
      x2 → Yanaşma    (sway)   → nötr (1500)
      y2 → Derinlik   (heave)  → mesafe sensörü PID çıkışı

Kullanım:
  ros2 run rov_vision autonomous_driver
  ros2 launch rov_vision autonomous_driver.launch.py
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import serial
import time
import threading

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge

import cv2
import numpy as np
from collections import deque
import math

# Mevcut line_follower.py'deki sınıfları doğrudan import ediyoruz
# (hiçbir şey değiştirilmedi — aynı PID, aynı görüntü işleme, aynı dedektör)
from rov_vision.line_follower import (
    PIDController,
    UnderwaterImageProcessor,
    LineDetector,
)


# =============================================================================
# YARDIMCI FONKSİYONLAR (Manuel sürüş koduyla birebir aynı)
# =============================================================================
def map_value(val, in_min, in_max, out_min, out_max):
    """Manuel sürüş kodundaki map fonksiyonu — değiştirilmedi."""
    return int((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


def clamp(val, lo, hi):
    """Değeri sınırlar içinde tutar."""
    return max(lo, min(hi, val))


# =============================================================================
# AUTONOMOUS DRIVER V2 NODE
# =============================================================================
class AutonomousDriverNode(Node):
    """
    Yarışma uyumlu otonom sürüş node'u.
    
    Akış:
      Kamera → Görüntü İşleme → Çizgi Tespiti → PID → Twist
      Mesafe Sensörü → Distance PID → Twist.linear.z  
      Twist → PWM → Serial → Arduino/STM32
      
    Serial çıkış formatı (manuel sürüşle birebir aynı):
      "{x1},{y1},{x2},{y2}\n"
      Her değer 1060-1940 aralığında, nötr = 1500
    """

    # ── Durum Makinesi (V2 — 7 Durum) ──────────────────────────────────
    STATE_SEARCHING     = "SEARCHING"       # Çizgi aranıyor
    STATE_FOLLOWING     = "FOLLOWING"       # Çizgi aktif takip ediliyor
    STATE_RECOVERING    = "RECOVERING"      # Çizgi geçici kayıp, kurtarma
    STATE_END_OF_LINE   = "END_OF_LINE"     # Hat sonu tespit edildi
    STATE_MISSION_READY = "MISSION_READY"   # Mini ROV bırakma hazır
    STATE_LOST          = "LOST"            # Çizgi tamamen kayıp
    STATE_EMERGENCY     = "EMERGENCY"       # Acil — tahtaya çok yakın!

    # ── PWM Sınırları (Manuel sürüş ile birebir aynı) ──────────────────
    PWM_MIN     = 1060
    PWM_MAX     = 1940
    PWM_NEUTRAL = 1500

    def __init__(self):
        super().__init__('autonomous_driver')

        # ── ROS2 Parametreleri ──────────────────────────────────────────
        self._declare_all_parameters()
        p = self._get_params()
        self.p = p

        # ── Serial Port (Manuel sürüş ile aynı ayarlar) ────────────────
        self.ser = None
        self._open_serial(p['serial_port'], p['baud_rate'])

        # ── Alt Sistemler (line_follower.py'den) ────────────────────────
        self.img_processor = UnderwaterImageProcessor(
            temporal_buffer_size=p['temporal_buffer_size']
        )
        self.detector = LineDetector(
            hsv_lower=tuple(p['hsv_lower']),
            hsv_upper=tuple(p['hsv_upper']),
            min_contour_area=p['min_contour_area'],
            roi_top_ratio=p['roi_top_ratio'],
            min_aspect_ratio=p['min_aspect_ratio']
        )

        # ── PID Kontrolcüleri ───────────────────────────────────────────
        # 1. Yanal hata PID (çizginin merkezden sapması)
        self.lateral_pid = PIDController(
            kp=p['pid_kp'], ki=p['pid_ki'], kd=p['pid_kd'],
            output_min=-p['max_angular_z'], output_max=p['max_angular_z'],
            integral_limit=p['pid_integral_limit']
        )
        # 2. Açı düzeltme PID (çizginin eğimi)
        self.angle_pid = PIDController(
            kp=p['angle_kp'], ki=0.0, kd=p['angle_kd'],
            output_min=-p['max_angular_z'] * 0.5,
            output_max=p['max_angular_z'] * 0.5
        )
        # 3. ✅ YENİ: Mesafe PID (tahtadan mesafe kontrolü — dikey eksen)
        self.distance_pid = PIDController(
            kp=p['distance_pid_kp'],
            ki=p['distance_pid_ki'],
            kd=p['distance_pid_kd'],
            output_min=-p['max_vertical_speed'],
            output_max=p['max_vertical_speed'],
            integral_limit=p['distance_pid_integral_limit']
        )

        # ── Durum Değişkenleri ──────────────────────────────────────────
        self.state = self.STATE_SEARCHING
        self.last_cx = None
        self.last_error = 0.0
        self.lost_counter = 0
        self.bridge = CvBridge()

        # ✅ YENİ: Takip sayacı (hat sonu algılama için)
        self.following_counter = 0

        # ✅ YENİ: Mesafe sensörü durumu
        self._current_distance_cm = -1.0    # -1 = henüz veri yok
        self._last_valid_distance_cm = -1.0
        self._distance_age_frames = 0       # Kaç frame'dir yeni mesafe gelmedi
        self._distance_last_time = 0.0

        # ✅ YENİ: Acil durum sayacı
        self._emergency_counter = 0
        self._emergency_pullback_frames = 0

        # ✅ YENİ: Hat sonu stabilizasyon zamanlayıcısı
        self._end_of_line_start_time = None

        # ── QoS ─────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── Subscriber'lar ──────────────────────────────────────────────
        self.create_subscription(
            Image,
            p['camera_topic'],
            self._image_callback,
            sensor_qos
        )
        # ✅ YENİ: Mesafe sensörü aboneliği (depth yerine distance)
        self.create_subscription(
            Float32,
            p['distance_topic'],
            self._distance_callback,
            sensor_qos
        )

        # ── Publisher'lar ───────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, p['cmd_vel_topic'], 10)
        self.debug_pub = self.create_publisher(Image, '/line_follower/debug_image', 1)
        self.status_pub = self.create_publisher(String, '/autonomous_driver/status', 10)

        # ── Watchdog ────────────────────────────────────────────────────
        self._last_image_time = time.monotonic()
        self.create_timer(1.0, self._watchdog_callback)

        self.get_logger().info(
            '🤖 Autonomous Driver V2 (Yarışma Modu) başlatıldı!\n'
            f'  Serial port     : {p["serial_port"]} @ {p["baud_rate"]} baud\n'
            f'  PWM aralığı     : {self.PWM_MIN}-{self.PWM_MAX} (nötr: {self.PWM_NEUTRAL})\n'
            f'  Mesafe sensörü  : {p["distance_topic"]}\n'
            f'  Hedef mesafe    : {p["target_distance_cm"]} cm\n'
            f'  Kritik mesafe   : {p["critical_distance_cm"]} cm\n'
            f'  Hat sonu eşiği  : {p["end_of_line_lost_frames"]} frame\n'
            f'  Min takip süresi: {p["min_following_before_eol"]} frame\n'
            f'  Kamera topic    : {p["camera_topic"]}\n'
            f'  HSV Alt         : {p["hsv_lower"]}\n'
            f'  HSV Üst         : {p["hsv_upper"]}'
        )

    # ═════════════════════════════════════════════════════════════════════
    #  PARAMETRE YÖNETİMİ
    # ═════════════════════════════════════════════════════════════════════
    def _declare_all_parameters(self):
        defaults = {
            # === Serial port (manuel sürüş ile aynı) ===
            'serial_port':    'COM8',
            'baud_rate':      115200,

            # === Topic'ler ===
            'camera_topic':   '/camera/image_raw',
            'cmd_vel_topic':  '/cmd_vel',
            'distance_topic': '/distance_sensor',   # ✅ YENİ (eski: depth_topic)

            # === Hız sınırları ===
            'linear_speed':       0.15,     # m/s (ileri/geri max hız)
            'max_angular_z':      0.8,      # rad/s (dönüş max hız)
            'max_vertical_speed': 0.3,      # ✅ YENİ: dikey eksen max hız

            # === PID - yanal hata (çizgi merkezden sapması) ===
            'pid_kp':             0.003,
            'pid_ki':             0.0001,
            'pid_kd':             0.001,
            'pid_integral_limit': 100.0,

            # === PID - açı düzeltme ===
            'angle_kp':           0.005,
            'angle_kd':           0.001,

            # === ✅ YENİ: PID - mesafe kontrolü (tahtadan mesafe) ===
            'distance_pid_kp':             0.008,
            'distance_pid_ki':             0.001,
            'distance_pid_kd':             0.003,
            'distance_pid_integral_limit': 50.0,
            'target_distance_cm':          25.0,    # Tahtadan hedef mesafe (cm)
            'critical_distance_cm':        10.0,    # Acil kaçış mesafesi (cm)
            'max_safe_distance_cm':        100.0,   # Bu üzerinde çizgi görülmez
            'invert_vertical':             False,   # Dikey yön ters ise True yap

            # === Görüntü işleme ===
            'temporal_buffer_size': 5,
            'roi_top_ratio':       0.4,
            'min_contour_area':    500,
            'min_aspect_ratio':    1.5,    # ✅ YENİ: Şerit en-boy oranı filtresi

            # === HSV eşikleri (siyah çizgi, kırmızı tahta üzerinde) ===
            'hsv_lower':           [0, 0, 0],
            'hsv_upper':           [180, 80, 60],

            # === Çizgi kayıp toleransı ===
            'max_lost_frames':     30,
            'search_angular_z':    0.3,

            # === ✅ YENİ: Hat sonu algılama ===
            'end_of_line_lost_frames':  20,     # Çizgi bu kadar frame kayıpsa hat bitti
            'min_following_before_eol': 60,     # Hat sonu demek için min takip süresi
            'eol_stabilize_seconds':    0.5,    # Hat sonunda kaç sn bekle → MISSION_READY (hızlı!)

            # === ✅ YENİ: Acil durum ===
            'emergency_pullback_frames': 10,    # Acil durumda kaç frame geri çekil
        }
        for name, val in defaults.items():
            self.declare_parameter(name, val)

    def _get_params(self):
        names = [
            'serial_port', 'baud_rate',
            'camera_topic', 'cmd_vel_topic', 'distance_topic',
            'linear_speed', 'max_angular_z', 'max_vertical_speed',
            'pid_kp', 'pid_ki', 'pid_kd', 'pid_integral_limit',
            'angle_kp', 'angle_kd',
            'distance_pid_kp', 'distance_pid_ki', 'distance_pid_kd',
            'distance_pid_integral_limit',
            'target_distance_cm', 'critical_distance_cm', 'max_safe_distance_cm',
            'invert_vertical',
            'temporal_buffer_size', 'roi_top_ratio', 'min_contour_area', 'min_aspect_ratio',
            'hsv_lower', 'hsv_upper',
            'max_lost_frames', 'search_angular_z',
            'end_of_line_lost_frames', 'min_following_before_eol',
            'eol_stabilize_seconds',
            'emergency_pullback_frames',
        ]
        return {n: self.get_parameter(n).value for n in names}

    # ═════════════════════════════════════════════════════════════════════
    #  SERIAL PORT YÖNETİMİ
    # ═════════════════════════════════════════════════════════════════════
    def _open_serial(self, port, baud):
        """Serial portu açar — manuel sürüş koduyla aynı parametreler."""
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'✅ [BAŞARILI] {port} portuna bağlanıldı.')
        except Exception as e:
            self.get_logger().error(
                f'❌ [HATA] Serial bağlantı kurulamadı: {e}\n'
                f'  Port: {port}, Baud: {baud}\n'
                f'  → Araç kontrolsüz çalışacak (sadece ROS2 cmd_vel yayını)'
            )
            self.ser = None

    def _send_serial_packet(self, x1, y1, x2, y2):
        """
        Manuel sürüş koduyla BİREBİR AYNI formatta serial paket gönderir.
        
        Paket: "{x1},{y1},{x2},{y2}\n"
        Her değer 1060-1940 arasında clamp edilir.
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

    # ═════════════════════════════════════════════════════════════════════
    #  TWIST → PWM DÖNÜŞÜMÜ
    # ═════════════════════════════════════════════════════════════════════
    def _twist_to_pwm(self, twist):
        """
        ROS2 Twist mesajını manuel sürüş PWM değerlerine çevirir.
        
        Eşleme (manuel sürüşteki joystick eksenleriyle birebir):
          twist.angular.z  → x1 (Dönüş)     : -max..+max → 1060..1940
          twist.linear.x   → y1 (İleri)      : -max..+max → 1940..1060 (ters!)
          twist.linear.y   → x2 (Yanaşma)    : -max..+max → 1060..1940
          twist.linear.z   → y2 (Derinlik)   : -max..+max → 1060..1940
        """
        max_lin = self.p['linear_speed']
        max_ang = self.p['max_angular_z']
        max_vert = self.p['max_vertical_speed']

        # Angular.z → x1 (dönüş)
        ang_z_clamped = clamp(twist.angular.z, -max_ang, max_ang)
        x1 = map_value(ang_z_clamped, -max_ang, max_ang, self.PWM_MAX, self.PWM_MIN)

        # Linear.x → y1 (ileri/geri) — joystick Y ekseni ters
        lin_x_clamped = clamp(twist.linear.x, -max_lin, max_lin)
        y1 = map_value(lin_x_clamped, -max_lin, max_lin, self.PWM_MAX, self.PWM_MIN)

        # Linear.y → x2 (yanaşma/strafe)
        lin_y_clamped = clamp(twist.linear.y, -max_lin, max_lin)
        x2 = map_value(lin_y_clamped, -max_lin, max_lin, self.PWM_MIN, self.PWM_MAX)

        # ✅ Linear.z → y2 (derinlik) — ARTIK mesafe PID tarafından kontrol ediliyor
        lin_z_clamped = clamp(twist.linear.z, -max_vert, max_vert)
        y2 = map_value(lin_z_clamped, -max_vert, max_vert, self.PWM_MIN, self.PWM_MAX)

        return x1, y1, x2, y2

    # ═════════════════════════════════════════════════════════════════════
    #  ✅ YENİ: MESAFE SENSÖRÜ CALLBACK
    # ═════════════════════════════════════════════════════════════════════
    def _distance_callback(self, msg: Float32):
        """
        Akustik mesafe sensöründen gelen veriyi alır (cm cinsinden).
        Aracın altından tahtaya olan mesafe.
        """
        self._current_distance_cm = msg.data
        self._last_valid_distance_cm = msg.data
        self._distance_age_frames = 0
        self._distance_last_time = time.monotonic()

    # ═════════════════════════════════════════════════════════════════════
    #  ✅ YENİ: DİKEY KONTROL (Mesafe PID)
    # ═════════════════════════════════════════════════════════════════════
    def _compute_vertical_control(self):
        """
        Mesafe sensörü verisine göre dikey itici kontrolü hesaplar.
        
        Mantık:
          - Hedef mesafeden yakınsa → YUKARI çık (tahtadan uzaklaş)
          - Hedef mesafeden uzaksa → AŞAĞI in (tahtaya yaklaş, çizgiyi gör)
          - Sensör verisi yoksa    → Nötr kal (güvenli)
        
        Returns:
          vertical_speed (float): twist.linear.z için değer
          is_emergency (bool): True ise acil durum, tüm motorları durdur
        """
        target = self.p['target_distance_cm']
        critical = self.p['critical_distance_cm']

        # Sensör verisi yoksa veya çok eskiyse → nötr
        if self._current_distance_cm < 0:
            return 0.0, False

        # Sensör verisi çok eskiyse (2 saniyeden fazla)
        if time.monotonic() - self._distance_last_time > 2.0:
            self.get_logger().warn('⚠️ Mesafe sensörü verisi eski! Dikey nötr.')
            return 0.0, False

        distance = self._current_distance_cm

        # ─── ACİL DURUM: Çok yakın! ─────────────────────────────────
        if distance < critical:
            self.get_logger().error(
                f'🚨 ACİL! Mesafe {distance:.1f} cm < {critical:.1f} cm! '
                f'YUKARI ÇEKİL!'
            )
            # Acil kaçış — max güçle yukarı
            emergency_speed = self.p['max_vertical_speed']
            if self.p['invert_vertical']:
                emergency_speed = -emergency_speed
            return emergency_speed, True

        # ─── Normal PID kontrolü ─────────────────────────────────────
        # Hata: hedef - gerçek
        # Pozitif hata = çok yakın → yukarı çık
        # Negatif hata = çok uzak → aşağı in
        error = target - distance

        pid_output = self.distance_pid.compute(error)

        # Yön tersine çevir (isteğe bağlı)
        if self.p['invert_vertical']:
            pid_output = -pid_output

        return pid_output, False

    # ═════════════════════════════════════════════════════════════════════
    #  ✅ YENİ: HAT SONU ALGILAMA
    # ═════════════════════════════════════════════════════════════════════
    def _check_end_of_line(self):
        """
        Çizginin gerçekten bitip bitmediğini kontrol eder.
        
        Hat sonu koşulları (TÜMÜ sağlanmalı):
          1. Çizgi kayıp süresi > end_of_line_lost_frames
          2. Daha önce yeterince uzun süre takip yapılmış olmalı
             (following_counter > min_following_before_eol)
        
        Bu sayede:
          - Geçici kayıplar (gölge, baloncuk) → RECOVERING (dön, ara)
          - Gerçek hat sonu → END_OF_LINE (dur, bekle)
        
        Returns:
          True: Hat sonu tespit edildi
          False: Henüz hat sonu değil
        """
        eol_threshold = self.p['end_of_line_lost_frames']
        min_follow = self.p['min_following_before_eol']

        if (self.lost_counter >= eol_threshold and
                self.following_counter >= min_follow):
            return True

        return False

    # ═════════════════════════════════════════════════════════════════════
    #  WATCHDOG
    # ═════════════════════════════════════════════════════════════════════
    def _watchdog_callback(self):
        dt = time.monotonic() - self._last_image_time
        if dt > 2.0:
            self.get_logger().warn(
                f'⚠️  {dt:.1f}s süredir görüntü yok! Motorlar nötre alınıyor.'
            )
            self._send_neutral()
            self._publish_zero_velocity()

    # ═════════════════════════════════════════════════════════════════════
    #  ANA GÖRÜNTÜ CALLBACK (V2 — Yarışma Uyumlu State Machine)
    # ═════════════════════════════════════════════════════════════════════
    def _image_callback(self, msg: Image):
        self._last_image_time = time.monotonic()
        self._distance_age_frames += 1

        # ── Eğer MISSION_READY veya END_OF_LINE ise hiçbir şey yapma ──
        if self.state == self.STATE_MISSION_READY:
            self._send_neutral()
            self._publish_zero_velocity()
            return

        if self.state == self.STATE_END_OF_LINE:
            self._send_neutral()
            self._publish_zero_velocity()
            # Stabilizasyon süresi doldu mu?
            if self._end_of_line_start_time is not None:
                elapsed = time.monotonic() - self._end_of_line_start_time
                if elapsed >= self.p['eol_stabilize_seconds']:
                    self.state = self.STATE_MISSION_READY
                    self.get_logger().info(
                        '\n'
                        '╔══════════════════════════════════════════════════╗\n'
                        '║  ✅ MINI ROV BIRAKMA HAZIR!                     ║\n'
                        '║  Araç durdu. Manuel kontrole geçebilirsiniz.    ║\n'
                        '║  Mini ROV\'u uzaktan kumanda ile aktif edin.     ║\n'
                        '╚══════════════════════════════════════════════════╝'
                    )
            return

        # ── ROS Image → OpenCV ────────────────────────────────────────
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge hatası: {e}')
            return

        h, w = frame.shape[:2]
        image_center_x = w // 2

        # ── Mesafe kontrolü (dikey eksen) ─────────────────────────────
        vertical_speed, is_emergency = self._compute_vertical_control()

        # ── ACİL DURUM: Tahtaya çok yakın! ────────────────────────────
        if is_emergency:
            self.state = self.STATE_EMERGENCY
            self._emergency_pullback_frames = self.p['emergency_pullback_frames']

        if self.state == self.STATE_EMERGENCY:
            # Acil durumda: ileri gitme, sadece yukarı çekil
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.z = vertical_speed  # Yukarı çekil
            twist.angular.z = 0.0

            x1, y1, x2, y2 = self._twist_to_pwm(twist)
            # İleri/dönüş motorlarını nötre al, sadece dikey aktif
            x1 = self.PWM_NEUTRAL
            y1 = self.PWM_NEUTRAL
            x2 = self.PWM_NEUTRAL
            self._send_serial_packet(x1, y1, x2, y2)
            self.cmd_pub.publish(twist)

            self._emergency_pullback_frames -= 1
            if self._emergency_pullback_frames <= 0:
                # Acil durum bitti, SEARCHING'e dön
                self.state = self.STATE_SEARCHING
                self.get_logger().info('🔄 Acil durum bitti, çizgi aramaya dönülüyor.')

            # Debug frame'e acil durum yaz
            cv2.putText(frame,
                        f'!!! ACIL - MESAFE: {self._current_distance_cm:.0f}cm !!!',
                        (10, h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            self._publish_debug(frame, msg.header)
            self._publish_status(None, twist, x1, y1, x2, y2)
            return

        # ── Su altı görüntü iyileştirme ──────────────────────────────
        enhanced = self.img_processor.enhance(frame)

        # ── Çizgi tespiti ────────────────────────────────────────────
        cx, cy, contour, debug_frame, angle_deg = self.detector.detect(enhanced)

        # ── State Machine (V2) ───────────────────────────────────────
        twist = Twist()

        if cx is not None:
            # ═══ ✅ ÇİZGİ BULUNDU ═══════════════════════════════════
            self.state = self.STATE_FOLLOWING
            self.lost_counter = 0
            self.following_counter += 1  # ✅ Takip sayacını artır
            self.last_cx = cx

            # Yatay hata (piksel)
            error_lateral = float(cx - image_center_x)
            self.last_error = error_lateral

            # Açı hatası
            angle_error = 0.0
            if abs(angle_deg) > 5.0:
                angle_error = angle_deg - 90.0
                if angle_error > 90.0:
                    angle_error -= 180.0

            # PID çıkışları
            angular_lateral = self.lateral_pid.compute(error_lateral)
            angular_angle = self.angle_pid.compute(angle_error)

            twist.linear.x = self.p['linear_speed']
            twist.angular.z = -(angular_lateral + angular_angle)

            # ✅ Dikey kontrol (mesafe PID çıkışı)
            twist.linear.z = vertical_speed

            # Debug HUD
            cv2.putText(debug_frame,
                        f'FOLLOWING | err:{error_lateral:.0f}px | '
                        f'ang_z:{twist.angular.z:.3f} | '
                        f'takip:{self.following_counter}',
                        (10, debug_frame.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        else:
            # ═══ ❌ ÇİZGİ BULUNAMADI ════════════════════════════════
            self.lost_counter += 1

            # ── Hat sonu kontrolü ────────────────────────────────────
            if self._check_end_of_line():
                # ✅ HAT SONU TESPİT EDİLDİ!
                self.state = self.STATE_END_OF_LINE
                self._end_of_line_start_time = time.monotonic()
                twist = Twist()  # Tamamen dur

                self.get_logger().info(
                    f'🏁 HAT SONU TESPİT EDİLDİ! '
                    f'(takip:{self.following_counter} frame, '
                    f'kayıp:{self.lost_counter} frame)\n'
                    f'  → {self.p["eol_stabilize_seconds"]:.1f}s stabilizasyon '
                    f'sonrası MISSION_READY\'e geçilecek.'
                )

                # PID'leri sıfırla
                self.lateral_pid.reset()
                self.angle_pid.reset()
                self.distance_pid.reset()

                cv2.putText(debug_frame,
                            'HAT SONU! Durduruluyor...',
                            (10, debug_frame.shape[0] // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3)

            elif self.lost_counter < self.p['max_lost_frames'] // 2:
                # ── RECOVERING: Son bilinen yöne hafif dön ───────────
                self.state = self.STATE_RECOVERING
                direction = np.sign(self.last_error) if self.last_error != 0 else 1.0
                twist.linear.x = 0.0
                twist.angular.z = direction * self.p['search_angular_z'] * 0.5
                twist.linear.z = vertical_speed  # Dikey kontrolü koru

            elif self.lost_counter < self.p['max_lost_frames']:
                # ── SEARCHING: Aktif arama dönüşü ────────────────────
                self.state = self.STATE_SEARCHING
                twist.linear.x = 0.0
                twist.angular.z = self.p['search_angular_z']
                twist.linear.z = vertical_speed  # Dikey kontrolü koru
                self.lateral_pid.reset()
                self.angle_pid.reset()

            else:
                # ── LOST: Tamamen dur ────────────────────────────────
                self.state = self.STATE_LOST
                twist = Twist()

            if self.state not in (self.STATE_END_OF_LINE, self.STATE_MISSION_READY):
                cv2.putText(debug_frame,
                            f'{self.state} | kayip:{self.lost_counter} | '
                            f'takip:{self.following_counter}',
                            (10, debug_frame.shape[0] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # ── Twist → PWM → Serial ─────────────────────────────────────
        x1, y1, x2, y2 = self._twist_to_pwm(twist)
        self._send_serial_packet(x1, y1, x2, y2)

        # ── ROS2 cmd_vel yayını ──────────────────────────────────────
        self.cmd_pub.publish(twist)

        # ── Debug görüntüsü ──────────────────────────────────────────
        # Mesafe bilgisini her zaman göster
        dist_color = (0, 255, 0)  # Yeşil = güvenli
        if self._current_distance_cm > 0:
            if self._current_distance_cm < self.p['critical_distance_cm']:
                dist_color = (0, 0, 255)  # Kırmızı = tehlikeli
            elif self._current_distance_cm < self.p['target_distance_cm'] * 0.7:
                dist_color = (0, 165, 255)  # Turuncu = dikkat

        cv2.putText(debug_frame,
                    f'PWM: {x1},{y1},{x2},{y2}',
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        dist_text = (f'Mesafe: {self._current_distance_cm:.1f}cm '
                     f'(hedef:{self.p["target_distance_cm"]:.0f}cm)'
                     if self._current_distance_cm > 0
                     else 'Mesafe: YOK')
        cv2.putText(debug_frame,
                    dist_text,
                    (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, dist_color, 2)

        # Durum çubuğu (üst kısım)
        state_colors = {
            self.STATE_FOLLOWING: (0, 255, 0),
            self.STATE_SEARCHING: (255, 255, 0),
            self.STATE_RECOVERING: (0, 165, 255),
            self.STATE_END_OF_LINE: (0, 255, 255),
            self.STATE_MISSION_READY: (0, 255, 0),
            self.STATE_LOST: (0, 0, 255),
            self.STATE_EMERGENCY: (0, 0, 255),
        }
        bar_color = state_colors.get(self.state, (128, 128, 128))
        cv2.rectangle(debug_frame, (0, 0), (w, 8), bar_color, -1)

        self._publish_debug(debug_frame, msg.header)
        self._publish_status(cx, twist, x1, y1, x2, y2)

    # ═════════════════════════════════════════════════════════════════════
    #  YARDIMCI METHODLAR
    # ═════════════════════════════════════════════════════════════════════
    def _send_neutral(self):
        """Tüm kanalları nötre al."""
        self._send_serial_packet(
            self.PWM_NEUTRAL, self.PWM_NEUTRAL,
            self.PWM_NEUTRAL, self.PWM_NEUTRAL
        )

    def _publish_zero_velocity(self):
        self.cmd_pub.publish(Twist())

    def _publish_debug(self, frame, header):
        """Debug görüntüsünü yayınlar."""
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            debug_msg.header = header
            self.debug_pub.publish(debug_msg)
        except Exception:
            pass

    def _publish_status(self, cx, twist, x1, y1, x2, y2):
        """Durum mesajını yayınlar."""
        status_msg = String()
        status_msg.data = (
            f'state={self.state},'
            f'cx={cx},'
            f'error={self.last_error:.1f},'
            f'distance_cm={self._current_distance_cm:.1f},'
            f'following={self.following_counter},'
            f'lost={self.lost_counter},'
            f'pwm={x1},{y1},{x2},{y2},'
            f'linear_x={twist.linear.x:.3f},'
            f'linear_z={twist.linear.z:.3f},'
            f'angular_z={twist.angular.z:.3f}'
        )
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        """Kapanırken serial portu kapat ve motorları nötre al."""
        self._send_neutral()
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial port kapatıldı.')
        super().destroy_node()


# =============================================================================
# MAIN
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Kullanıcı tarafından durduruldu.')
    finally:
        # Güvenli kapanış: motorları nötre al
        node._send_neutral()
        node._publish_zero_velocity()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
