#!/usr/bin/env python3
"""
=============================================================================
AUTONOMOUS DRIVER V3 - Firmware Uyumlu (Hat Takibi & Otonom Sürüş)
=============================================================================
Teknofest İnsansız Su Altı Sistemleri Yarışması — Antigravity Takımı

V2'den Değişiklikler:
  1. SERIAL PROTOKOL     : Firmware uyumlu "A,m1,m2,m3,m4,m5,m6,btn,kp,kd\n"
  2. THRUSTER MIX        : Vektörel motor karışımı Python tarafında yapılıyor
  3. MESAFE SENSÖRÜ      : Deneyap'tan serial ile okunan akustik sensör verisi
  4. BASINÇ SENSÖRÜ      : Jetson I2C (MS5837) → /depth_sensor ROS2 topic
  5. HAT SONU ALGILAMA   : Çizgi bitince tam dur
  6. ACİL ÇARPMA KORUMASI: Kritik mesafede tüm motorları kes, geri çekil

Durum Makinesi (State Machine):
  SEARCHING → FOLLOWING → RECOVERING → END_OF_LINE → MISSION_READY
                                      ↘ SEARCHING (kısa kayıp)
                                      ↘ LOST (uzun kayıp, takip geçmişi yok)
  EMERGENCY (mesafe kritik) → herhangi durumda tetiklenebilir

Serial Protokol (AnaROV_vehicle.ino firmware ile uyumlu):
  - Jetson → Deneyap TX: "A,m1,m2,m3,m4,m5,m6,btn,kp,kd\n"
  - Deneyap → Jetson RX: "D,mesafe_cm\n" (akustik sensör)
  - Motor değerleri: -100 ile +100 arası yüzdelik
  - btn: 0=normal, 1=acil durdurma

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

from sensor_msgs.msg import Image, Imu
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
# YARDIMCI FONKSİYONLAR
# =============================================================================
def clamp(val, lo, hi):
    """Değeri sınırlar içinde tutar."""
    return max(lo, min(hi, val))


# =============================================================================
# AUTONOMOUS DRIVER V3 NODE
# =============================================================================
class AutonomousDriverNode(Node):
    """
    Yarışma uyumlu otonom sürüş node'u.
    
    Akış:
      Kamera → Görüntü İşleme → Çizgi Tespiti → PID → Yön Komutları
      Yön Komutları → Thruster Mix → 6 Motor Yüzdesi → Serial → Deneyap
      Deneyap → Akustik Mesafe → Serial → Bu Node → Dikey PID
      MS5837 → I2C → pressure_publisher → /depth_sensor → Bu Node
      
    Serial çıkış formatı (AnaROV_vehicle.ino firmware ile uyumlu):
      "A,m1,m2,m3,m4,m5,m6,btn,kp,kd\n"
      Motor değerleri: -100 ile +100 arası yüzdelik
    """

    # ── Durum Makinesi (V2 — 7 Durum) ──────────────────────────────────
    STATE_SEARCHING     = "SEARCHING"       # Çizgi aranıyor
    STATE_FOLLOWING     = "FOLLOWING"       # Çizgi aktif takip ediliyor
    STATE_RECOVERING    = "RECOVERING"      # Çizgi geçici kayıp, kurtarma
    STATE_END_OF_LINE   = "END_OF_LINE"     # Hat sonu tespit edildi
    STATE_MISSION_READY = "MISSION_READY"   # Mini ROV bırakma hazır
    STATE_LOST          = "LOST"            # Çizgi tamamen kayıp
    STATE_EMERGENCY     = "EMERGENCY"       # Acil — tahtaya çok yakın!

    def __init__(self):
        super().__init__('autonomous_driver')

        # ── ROS2 Parametreleri ──────────────────────────────────────────
        self._declare_all_parameters()
        p = self._get_params()
        self.p = p

        # ── Serial Port (Deneyap kart ile çift yönlü iletişim) ────────
        self.ser = None
        self._open_serial(p['serial_port'], p['baud_rate'])

        # ── Serial okuma thread'i (Deneyap'tan mesafe verisi) ──────────
        self._running = True
        self._serial_thread = threading.Thread(
            target=self._serial_read_loop, daemon=True
        )
        self._serial_thread.start()

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
            output_min=-1.0, output_max=1.0,
            integral_limit=p['pid_integral_limit']
        )
        # 2. Açı düzeltme PID (çizginin eğimi)
        self.angle_pid = PIDController(
            kp=p['angle_kp'], ki=0.0, kd=p['angle_kd'],
            output_min=-0.5, output_max=0.5
        )
        # 3. Mesafe PID (tahtadan mesafe kontrolü — dikey eksen)
        self.distance_pid = PIDController(
            kp=p['distance_pid_kp'],
            ki=p['distance_pid_ki'],
            kd=p['distance_pid_kd'],
            output_min=-1.0,
            output_max=1.0,
            integral_limit=p['distance_pid_integral_limit']
        )
        
        # 4. Roll PID (IMU'dan hesaplanan yatıklık açısı için)
        self.roll_pid = PIDController(
            kp=p['roll_kp'], ki=0.0, kd=p['roll_kd'],
            output_min=-0.4, output_max=0.4
        )

        # ── Durum Değişkenleri ──────────────────────────────────────────
        self.state = self.STATE_SEARCHING
        self.last_cx = None
        self.last_error = 0.0
        self.lost_counter = 0
        self.bridge = CvBridge()

        # Takip sayacı (hat sonu algılama için)
        self.following_counter = 0

        # Mesafe sensörü durumu (Deneyap serial'den okunuyor)
        self._current_distance_cm = -1.0    # -1 = henüz veri yok
        self._last_valid_distance_cm = -1.0
        self._distance_last_time = 0.0
        self._serial_lock = threading.Lock()

        # Basınç sensörü durumu (ROS2 topic'ten okunuyor)
        self._current_depth_cm = -1.0
        self._depth_last_time = 0.0

        # IMU Roll durumu
        self._current_roll_deg = 0.0

        # Acil durum sayacı
        self._emergency_counter = 0
        self._emergency_pullback_frames = 0

        # Hat sonu stabilizasyon zamanlayıcısı
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
        # Basınç sensörü (MS5837 → pressure_publisher → /depth_sensor)
        self.create_subscription(
            Float32,
            p['depth_topic'],
            self._depth_callback,
            sensor_qos
        )
        # Mesafe sensörü (Jetson UART → distance_publisher → /distance_sensor)
        self.create_subscription(
            Float32,
            '/distance_sensor',
            self._distance_callback,
            sensor_qos
        )
        # IMU sensörü (Jetson I2C → imu_publisher → /imu/data)
        self.create_subscription(
            Imu,
            '/imu/data',
            self._imu_callback,
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
            '🤖 Autonomous Driver V3 (Firmware Uyumlu) başlatıldı!\n'
            f'  Serial port     : {p["serial_port"]} @ {p["baud_rate"]} baud\n'
            f'  Protokol        : A,m1,m2,m3,m4,m5,m6,btn,kp,kd\n'
            f'  Güç sınırı      : %{p["power_limit"]}\n'
            f'  Mesafe sensörü  : Deneyap serial (D,cm)\n'
            f'  Basınç sensörü  : {p["depth_topic"]}\n'
            f'  Hedef mesafe    : {p["target_distance_cm"]} cm\n'
            f'  Kritik mesafe   : {p["critical_distance_cm"]} cm\n'
            f'  Kamera topic    : {p["camera_topic"]}\n'
            f'  HSV Alt         : {p["hsv_lower"]}\n'
            f'  HSV Üst         : {p["hsv_upper"]}'
        )

    # ═════════════════════════════════════════════════════════════════════
    #  PARAMETRE YÖNETİMİ
    # ═════════════════════════════════════════════════════════════════════
    def _declare_all_parameters(self):
        defaults = {
            # === Serial port ===
            'serial_port':    '/dev/ttyUSB0',
            'baud_rate':      115200,

            # === Topic'ler ===
            'camera_topic':   '/camera/image_raw',
            'cmd_vel_topic':  '/cmd_vel',
            'depth_topic':    '/depth_sensor',   # MS5837 basınç sensörü

            # === Motor kontrol ===
            'power_limit':        50,       # Maksimum motor gücü (%)
            'linear_speed':       0.40,     # İleri hız (0.0-1.0 arası normalize)
            'max_angular_z':      0.8,      # Maksimum dönüş hızı (normalize)
            'max_vertical_speed': 0.3,      # Dikey eksen max hız (normalize)
            'search_angular_z':   0.3,      # Arama dönüş hızı (normalize)

            # === Roll PID (Deneyap firmware'ine gönderilir) ===
            'roll_kp':            1.5,
            'roll_kd':            0.25,

            # === PID - yanal hata (çizgi merkezden sapması) ===
            'pid_kp':             0.003,
            'pid_ki':             0.0001,
            'pid_kd':             0.001,
            'pid_integral_limit': 100.0,

            # === PID - açı düzeltme ===
            'angle_kp':           0.005,
            'angle_kd':           0.001,

            # === PID - mesafe kontrolü (tahtadan mesafe) ===
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
            'min_aspect_ratio':    1.5,

            # === HSV eşikleri (siyah çizgi, kırmızı tahta üzerinde) ===
            'hsv_lower':           [0, 0, 0],
            'hsv_upper':           [180, 50, 50],

            # === Çizgi kayıp toleransı ===
            'max_lost_frames':     30,

            # === Hat sonu algılama ===
            'end_of_line_lost_frames':  20,
            'min_following_before_eol': 60,
            'eol_stabilize_seconds':    0.5,

            # === Acil durum ===
            'emergency_pullback_frames': 10,
        }
        for name, val in defaults.items():
            self.declare_parameter(name, val)

    def _get_params(self):
        names = [
            'serial_port', 'baud_rate',
            'camera_topic', 'cmd_vel_topic', 'depth_topic',
            'power_limit', 'linear_speed', 'max_angular_z',
            'max_vertical_speed', 'search_angular_z',
            'roll_kp', 'roll_kd',
            'pid_kp', 'pid_ki', 'pid_kd', 'pid_integral_limit',
            'angle_kp', 'angle_kd',
            'distance_pid_kp', 'distance_pid_ki', 'distance_pid_kd',
            'distance_pid_integral_limit',
            'target_distance_cm', 'critical_distance_cm', 'max_safe_distance_cm',
            'invert_vertical',
            'temporal_buffer_size', 'roi_top_ratio', 'min_contour_area', 'min_aspect_ratio',
            'hsv_lower', 'hsv_upper',
            'max_lost_frames',
            'end_of_line_lost_frames', 'min_following_before_eol',
            'eol_stabilize_seconds',
            'emergency_pullback_frames',
        ]
        return {n: self.get_parameter(n).value for n in names}

    # ═════════════════════════════════════════════════════════════════════
    #  SERIAL PORT YÖNETİMİ (Çift Yönlü — TX: motor, RX: mesafe)
    # ═════════════════════════════════════════════════════════════════════
    def _open_serial(self, port, baud):
        """Serial portu açar — Deneyap kart ile çift yönlü iletişim."""
        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info(f'✅ [BAŞARILI] {port} portuna bağlanıldı.')
        except Exception as e:
            self.get_logger().error(
                f'❌ [HATA] Serial bağlantı kurulamadı: {e}\n'
                f'  Port: {port}, Baud: {baud}\n'
                f'  → Araç kontrolsüz çalışacak (sadece ROS2 cmd_vel yayını)'
            )
            self.ser = None

    def _serial_read_loop(self):
        """
        Arka plan thread'i: Deneyap'tan gelen seri port verisini okur.
        (Mesafe artık ROS2 topic üzerinden geldiği için sadece debug/log okur)
        """
        while self._running:
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line and not line.startswith("A,"):
                        self.get_logger().debug(f'Deneyap: {line}')
            except Exception:
                pass
            time.sleep(0.005)  # 200Hz polling — hızlı tepki

    # ═════════════════════════════════════════════════════════════════════
    #  VEKTÖREL THRUSTER MİX (AnaROV_video_mission.ino ile aynı mantık)
    # ═════════════════════════════════════════════════════════════════════
    def _thruster_mix(self, fwd, strafe, yaw, dive):
        """
        Yön komutlarını 6 motor yüzdesine çevirir.
        
        AnaROV_video_mission.ino'daki thrusterMix() fonksiyonu ile
        BİREBİR AYNI motor karışım formülleri kullanılır.
        
        Args:
            fwd:    İleri/geri  (-1.0 .. +1.0)
            strafe: Sağ/sol     (-1.0 .. +1.0)
            yaw:    Dönüş       (-1.0 .. +1.0)
            dive:   Dalış/çıkış (-1.0 .. +1.0)
        
        Returns:
            (m1, m2, m3, m4, m5, m6) — her biri -100..+100 arası yüzde
        """
        POWER_LIMIT = self.p['power_limit']

        # Jetson tarafında Roll PID hesaplaması (Hedef 0 derece = tam düz)
        roll_correction = self.roll_pid.compute(0.0 - self._current_roll_deg)

        # Yatay motorlar (4 adet — vektörel karışım)
        m_fr = fwd - strafe - yaw       # M1: Ön Sağ
        m_fl = fwd + strafe + yaw       # M2: Ön Sol
        m_rr = fwd + strafe - yaw       # M3: Arka Sağ
        m_rl = fwd - strafe + yaw       # M4: Arka Sol

        # Dikey motorlar (2 adet — aynı değer + roll düzeltmesi)
        m_vl = dive - roll_correction   # M5: Dikey Sol
        m_vr = dive + roll_correction   # M6: Dikey Sağ

        # Güç sınırı uygula ve yüzdeye çevir (-100..+100)
        m1 = int(clamp(m_fr * POWER_LIMIT, -100, 100))
        m2 = int(clamp(m_fl * POWER_LIMIT, -100, 100))
        m3 = int(clamp(m_rr * POWER_LIMIT, -100, 100))
        m4 = int(clamp(m_rl * POWER_LIMIT, -100, 100))
        m5 = int(clamp(m_vl * POWER_LIMIT, -100, 100))
        m6 = int(clamp(m_vr * POWER_LIMIT, -100, 100))

        return m1, m2, m3, m4, m5, m6

    # ═════════════════════════════════════════════════════════════════════
    #  SERIAL PAKET GÖNDERİMİ (AnaROV_vehicle.ino firmware uyumlu)
    # ═════════════════════════════════════════════════════════════════════
    def _send_serial_packet(self, m1, m2, m3, m4, m5, m6, btn=0):
        """
        AnaROV_vehicle.ino firmware'ine motor komutu gönderir.
        
        Firmware format: "A,m1,m2,m3,m4,m5,m6,btn,kp,kd\n"
        - m1..m6: Motor yüzdeleri (-100 ile +100 arası)
        - btn: Kill switch (0=normal, 1=acil durdurma)
        - kp, kd: 0, 0 (Roll kontrolü artık Jetson'da yapıldığı için Deneyap'a 0 gidiyor)
        """
        paket = f"A,{m1},{m2},{m3},{m4},{m5},{m6},{btn},0,0\n"

        if self.ser and self.ser.is_open:
            try:
                self.ser.write(paket.encode('utf-8'))
            except Exception as e:
                self.get_logger().warn(f'Serial yazma hatası: {e}')

    def _send_neutral(self):
        """Tüm motorları nötre al (0% güç)."""
        self._send_serial_packet(0, 0, 0, 0, 0, 0, btn=0)

    def _send_kill(self):
        """Acil durdurma — firmware kill switch aktif."""
        self._send_serial_packet(0, 0, 0, 0, 0, 0, btn=1)

    # ═════════════════════════════════════════════════════════════════════
    #  BASINÇ SENSÖRÜ CALLBACK (MS5837 → /depth_sensor)
    # ═════════════════════════════════════════════════════════════════════
    def _depth_callback(self, msg: Float32):
        """
        MS5837 basınç sensöründen gelen derinlik verisini alır (cm cinsinden).
        pressure_publisher node'u tarafından yayınlanır.
        """
        self._current_depth_cm = msg.data
        self._depth_last_time = time.monotonic()

    def _distance_callback(self, msg: Float32):
        """
        Jetson UART üzerinden bağlanan akustik mesafe sensörü verisini alır (cm).
        """
        with self._serial_lock:
            self._current_distance_cm = msg.data
            self._last_valid_distance_cm = msg.data
            self._distance_last_time = time.monotonic()

    def _imu_callback(self, msg: Imu):
        """
        Jetson I2C üzerinden bağlanan IMU verisinden anlık Roll (yatıklık) açısını hesaplar.
        """
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        # Y ve Z ivmesine göre Roll açısı (radyandan dereceye çevrilir)
        roll_rad = math.atan2(ay, az)
        self._current_roll_deg = math.degrees(roll_rad)

    # ═════════════════════════════════════════════════════════════════════
    #  DİKEY KONTROL (Mesafe PID — Tahtaya Çarpmama)
    # ═════════════════════════════════════════════════════════════════════
    def _compute_vertical_control(self):
        """
        Akustik mesafe sensörü verisine göre dikey itici kontrolü hesaplar.
        
        Mantık:
          - Hedef mesafeden yakınsa → YUKARI çık (tahtadan uzaklaş)
          - Hedef mesafeden uzaksa → AŞAĞI in (tahtaya yaklaş, çizgiyi gör)
          - Sensör verisi yoksa    → Nötr kal (güvenli)
        
        Returns:
          vertical_speed (float): -1.0..+1.0 arası dikey hız komutu
          is_emergency (bool): True ise acil durum, tüm motorları durdur
        """
        target = self.p['target_distance_cm']
        critical = self.p['critical_distance_cm']

        with self._serial_lock:
            distance = self._current_distance_cm
            last_time = self._distance_last_time

        # Sensör verisi yoksa → nötr
        if distance < 0:
            return 0.0, False

        # Sensör verisi çok eskiyse (2 saniyeden fazla)
        if time.monotonic() - last_time > 2.0:
            self.get_logger().warn('⚠️ Mesafe sensörü verisi eski! Dikey nötr.')
            return 0.0, False

        # ─── ACİL DURUM: Çok yakın! ─────────────────────────────────
        if distance < critical:
            self.get_logger().error(
                f'🚨 ACİL! Mesafe {distance:.1f} cm < {critical:.1f} cm! '
                f'YUKARI ÇEKİL!'
            )
            emergency_speed = 1.0  # Maksimum güçle yukarı
            if self.p['invert_vertical']:
                emergency_speed = -emergency_speed
            return emergency_speed, True

        # ─── Normal PID kontrolü ─────────────────────────────────────
        error = target - distance
        pid_output = self.distance_pid.compute(error)

        if self.p['invert_vertical']:
            pid_output = -pid_output

        return pid_output, False

    # ═════════════════════════════════════════════════════════════════════
    #  HAT SONU ALGILAMA
    # ═════════════════════════════════════════════════════════════════════
    def _check_end_of_line(self):
        """
        Çizginin gerçekten bitip bitmediğini kontrol eder.
        
        Hat sonu koşulları (TÜMÜ sağlanmalı):
          1. Çizgi kayıp süresi > end_of_line_lost_frames
          2. Daha önce yeterince uzun süre takip yapılmış olmalı
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
    #  ANA GÖRÜNTÜ CALLBACK (V3 — Firmware Uyumlu State Machine)
    # ═════════════════════════════════════════════════════════════════════
    def _image_callback(self, msg: Image):
        self._last_image_time = time.monotonic()

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

        # ── Mesafe kontrolü (dikey eksen — akustik sensör) ────────────
        vertical_speed, is_emergency = self._compute_vertical_control()

        # ── ACİL DURUM: Tahtaya çok yakın! ────────────────────────────
        if is_emergency:
            self.state = self.STATE_EMERGENCY
            self._emergency_pullback_frames = self.p['emergency_pullback_frames']

        if self.state == self.STATE_EMERGENCY:
            # Acil durumda: ileri gitme, sadece yukarı çekil
            m1, m2, m3, m4, m5, m6 = self._thruster_mix(
                fwd=0.0, strafe=0.0, yaw=0.0, dive=vertical_speed
            )
            self._send_serial_packet(m1, m2, m3, m4, m5, m6)

            # ROS2 cmd_vel (telemetri için)
            twist = Twist()
            twist.linear.z = vertical_speed
            self.cmd_pub.publish(twist)

            self._emergency_pullback_frames -= 1
            if self._emergency_pullback_frames <= 0:
                self.state = self.STATE_SEARCHING
                self.get_logger().info('🔄 Acil durum bitti, çizgi aramaya dönülüyor.')

            # Debug frame'e acil durum yaz
            with self._serial_lock:
                dist = self._current_distance_cm
            cv2.putText(frame,
                        f'!!! ACIL - MESAFE: {dist:.0f}cm !!!',
                        (10, h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            self._publish_debug(frame, msg.header)
            self._publish_status(None, twist, m1, m2, m3, m4, m5, m6)
            return

        # ── Su altı görüntü iyileştirme ──────────────────────────────
        enhanced = self.img_processor.enhance(frame)

        # ── Çizgi tespiti ────────────────────────────────────────────
        cx, cy, contour, debug_frame, angle_deg = self.detector.detect(enhanced)

        # ── State Machine (V3) ───────────────────────────────────────
        twist = Twist()

        # Yön komutları (normalize: -1.0 .. +1.0)
        fwd_cmd = 0.0
        yaw_cmd = 0.0
        strafe_cmd = 0.0
        dive_cmd = vertical_speed  # Mesafe PID çıkışı her zaman aktif

        if cx is not None:
            # ═══ ✅ ÇİZGİ BULUNDU ═══════════════════════════════════
            self.state = self.STATE_FOLLOWING
            self.lost_counter = 0
            self.following_counter += 1
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

            # PID çıkışları (normalize -1.0..+1.0)
            angular_lateral = self.lateral_pid.compute(error_lateral)
            angular_angle = self.angle_pid.compute(angle_error)

            fwd_cmd = self.p['linear_speed']
            yaw_cmd = -(angular_lateral + angular_angle)

            # ROS2 Twist (telemetri/kayıt için)
            twist.linear.x = fwd_cmd
            twist.angular.z = yaw_cmd
            twist.linear.z = dive_cmd

            # Debug HUD
            cv2.putText(debug_frame,
                        f'FOLLOWING | err:{error_lateral:.0f}px | '
                        f'yaw:{yaw_cmd:.3f} | '
                        f'takip:{self.following_counter}',
                        (10, debug_frame.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        else:
            # ═══ ❌ ÇİZGİ BULUNAMADI ════════════════════════════════
            self.lost_counter += 1

            # ── Hat sonu kontrolü ────────────────────────────────────
            if self._check_end_of_line():
                self.state = self.STATE_END_OF_LINE
                self._end_of_line_start_time = time.monotonic()
                # Tamamen dur
                fwd_cmd = 0.0
                yaw_cmd = 0.0
                dive_cmd = 0.0

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
                fwd_cmd = 0.0
                yaw_cmd = direction * self.p['search_angular_z'] * 0.5

            elif self.lost_counter < self.p['max_lost_frames']:
                # ── SEARCHING: Aktif arama dönüşü ────────────────────
                self.state = self.STATE_SEARCHING
                fwd_cmd = 0.0
                yaw_cmd = self.p['search_angular_z']
                self.lateral_pid.reset()
                self.angle_pid.reset()

            else:
                # ── LOST: Tamamen dur ────────────────────────────────
                self.state = self.STATE_LOST
                fwd_cmd = 0.0
                yaw_cmd = 0.0
                dive_cmd = 0.0

            twist.linear.x = fwd_cmd
            twist.angular.z = yaw_cmd
            twist.linear.z = dive_cmd

            if self.state not in (self.STATE_END_OF_LINE, self.STATE_MISSION_READY):
                cv2.putText(debug_frame,
                            f'{self.state} | kayip:{self.lost_counter} | '
                            f'takip:{self.following_counter}',
                            (10, debug_frame.shape[0] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # ── Thruster Mix → Serial ─────────────────────────────────────
        m1, m2, m3, m4, m5, m6 = self._thruster_mix(
            fwd=fwd_cmd, strafe=strafe_cmd, yaw=yaw_cmd, dive=dive_cmd
        )
        self._send_serial_packet(m1, m2, m3, m4, m5, m6)

        # ── ROS2 cmd_vel yayını (telemetri) ───────────────────────────
        self.cmd_pub.publish(twist)

        # ── Debug görüntüsü ──────────────────────────────────────────
        with self._serial_lock:
            dist = self._current_distance_cm

        # Motor değerleri
        cv2.putText(debug_frame,
                    f'M: {m1},{m2},{m3},{m4},{m5},{m6}',
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # Mesafe bilgisi
        dist_color = (0, 255, 0)
        if dist > 0:
            if dist < self.p['critical_distance_cm']:
                dist_color = (0, 0, 255)
            elif dist < self.p['target_distance_cm'] * 0.7:
                dist_color = (0, 165, 255)

        dist_text = (f'Mesafe: {dist:.1f}cm '
                     f'(hedef:{self.p["target_distance_cm"]:.0f}cm)'
                     if dist > 0
                     else 'Mesafe: SENSÖR YOK')
        cv2.putText(debug_frame,
                    dist_text,
                    (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, dist_color, 2)

        # Derinlik bilgisi (basınç sensörü)
        depth_text = (f'Derinlik: {self._current_depth_cm:.1f}cm'
                      if self._current_depth_cm > 0
                      else 'Derinlik: SENSÖR YOK')
        cv2.putText(debug_frame,
                    depth_text,
                    (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 2)

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
        self._publish_status(cx, twist, m1, m2, m3, m4, m5, m6)

    # ═════════════════════════════════════════════════════════════════════
    #  YARDIMCI METHODLAR
    # ═════════════════════════════════════════════════════════════════════
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

    def _publish_status(self, cx, twist, m1, m2, m3, m4, m5, m6):
        """Durum mesajını yayınlar."""
        with self._serial_lock:
            dist = self._current_distance_cm
        status_msg = String()
        status_msg.data = (
            f'state={self.state},'
            f'cx={cx},'
            f'error={self.last_error:.1f},'
            f'distance_cm={dist:.1f},'
            f'depth_cm={self._current_depth_cm:.1f},'
            f'following={self.following_counter},'
            f'lost={self.lost_counter},'
            f'motors={m1},{m2},{m3},{m4},{m5},{m6},'
            f'linear_x={twist.linear.x:.3f},'
            f'linear_z={twist.linear.z:.3f},'
            f'angular_z={twist.angular.z:.3f}'
        )
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        """Kapanırken serial portu kapat ve motorları nötre al."""
        self._running = False  # Serial okuma thread'ini durdur
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
