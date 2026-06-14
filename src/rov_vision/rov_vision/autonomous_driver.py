#!/usr/bin/env python3
"""
=============================================================================
AUTONOMOUS DRIVER - Çizgi Takip → Serial PWM Köprüsü
=============================================================================
Bu modül, mevcut line_follower.py'nin kamera+görüntü işleme çıkışını alıp
manuel sürüş kodundaki ile BİREBİR AYNI serial paket formatında
(x1,y1,x2,y2\n  /  1060-1940 PWM aralığı) Arduino/STM32'ye gönderir.

Manuel sürüş kodundaki hiçbir protokol değişmez:
  - Serial port: COM8 (veya parametre ile ayarlanır)
  - Baud rate: 115200
  - Paket formatı: "x1,y1,x2,y2\n"
  - PWM aralığı: 1060-1940 (nötr: 1500)
  - Güncelleme: ~20 Hz

Kanal eşlemeleri (manuel sürüşteki ile aynı):
  x1 → Sol Dön      (yaw/dönüş)     → çizgi takipten gelen angular.z
  y1 → Sol İleri     (ileri/geri)     → çizgi takipten gelen linear.x
  x2 → Sağ Yanaş     (lateral/strafe) → şu an nötr (1500)
  y2 → Sağ Derinlik  (dikey)          → şu an nötr (1500), ileride derinlik kontrolü

Kullanım:
  ros2 run rov_vision autonomous_driver
  ros2 launch rov_vision line_follower.launch.py
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
# YARDIMCI: PWM Mapping (Manuel sürüş koduyla birebir aynı)
# =============================================================================
def map_value(val, in_min, in_max, out_min, out_max):
    """Manuel sürüş kodundaki map fonksiyonu — değiştirilmedi."""
    return int((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


def clamp(val, lo, hi):
    """Değeri sınırlar içinde tutar."""
    return max(lo, min(hi, val))


# =============================================================================
# AUTONOMOUS DRIVER NODE
# =============================================================================
class AutonomousDriverNode(Node):
    """
    Çizgi takip sonuçlarını serial PWM komutlarına çeviren ROS2 node'u.
    
    Akış:
      Kamera → Görüntü İşleme → Çizgi Tespiti → PID → Twist → PWM → Serial
      
    Serial çıkış formatı (manuel sürüşle birebir aynı):
      "{x1},{y1},{x2},{y2}\n"
      Her değer 1060-1940 aralığında, nötr = 1500
    """

    # State Machine (line_follower.py ile aynı)
    STATE_SEARCHING  = "SEARCHING"
    STATE_FOLLOWING  = "FOLLOWING"
    STATE_RECOVERING = "RECOVERING"
    STATE_LOST       = "LOST"

    # PWM sınırları (manuel sürüş ile birebir aynı)
    PWM_MIN    = 1060
    PWM_MAX    = 1940
    PWM_NEUTRAL = 1500

    def __init__(self):
        super().__init__('autonomous_driver')

        # ── ROS2 Parametreleri ──────────────────────────────────────────────
        self._declare_all_parameters()
        p = self._get_params()
        self.p = p

        # ── Serial Port (Manuel sürüş ile aynı ayarlar) ────────────────────
        self.ser = None
        self._open_serial(p['serial_port'], p['baud_rate'])

        # ── Alt sistemler (line_follower.py ile birebir aynı) ───────────────
        self.img_processor = UnderwaterImageProcessor(
            temporal_buffer_size=p['temporal_buffer_size']
        )
        self.detector = LineDetector(
            hsv_lower=tuple(p['hsv_lower']),
            hsv_upper=tuple(p['hsv_upper']),
            min_contour_area=p['min_contour_area'],
            roi_top_ratio=p['roi_top_ratio']
        )
        self.lateral_pid = PIDController(
            kp=p['pid_kp'], ki=p['pid_ki'], kd=p['pid_kd'],
            output_min=-p['max_angular_z'], output_max=p['max_angular_z'],
            integral_limit=p['pid_integral_limit']
        )
        self.angle_pid = PIDController(
            kp=p['angle_kp'], ki=0.0, kd=p['angle_kd'],
            output_min=-p['max_angular_z'] * 0.5,
            output_max=p['max_angular_z'] * 0.5
        )

        # ── Durum değişkenleri ──────────────────────────────────────────────
        self.state = self.STATE_SEARCHING
        self.last_cx = None
        self.last_error = 0.0
        self.lost_counter = 0
        self.bridge = CvBridge()
        self._current_depth = 0.0

        # ── QoS ─────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── Subscriber'lar ──────────────────────────────────────────────────
        self.create_subscription(
            Image,
            p['camera_topic'],
            self._image_callback,
            sensor_qos
        )
        self.create_subscription(
            Float32,
            p['depth_topic'],
            self._depth_callback,
            sensor_qos
        )

        # ── Publisher'lar (debug amaçlı, cmd_vel de yayınlanır) ─────────────
        self.cmd_pub = self.create_publisher(Twist, p['cmd_vel_topic'], 10)
        self.debug_pub = self.create_publisher(Image, '/line_follower/debug_image', 1)
        self.status_pub = self.create_publisher(String, '/line_follower/status', 10)

        # ── Watchdog ────────────────────────────────────────────────────────
        self._last_image_time = time.monotonic()
        self.create_timer(1.0, self._watchdog_callback)

        self.get_logger().info(
            '🤖 Autonomous Driver başlatıldı!\n'
            f'  Serial port   : {p["serial_port"]} @ {p["baud_rate"]} baud\n'
            f'  PWM aralığı   : {self.PWM_MIN}-{self.PWM_MAX} (nötr: {self.PWM_NEUTRAL})\n'
            f'  Paket formatı : x1,y1,x2,y2\\n (manuel sürüşle aynı)\n'
            f'  Kamera topic  : {p["camera_topic"]}\n'
            f'  Hız (ileri)   : {p["linear_speed"]} m/s\n'
            f'  HSV Alt       : {p["hsv_lower"]}\n'
            f'  HSV Üst       : {p["hsv_upper"]}'
        )

    # ── Serial Port Yönetimi ─────────────────────────────────────────────────
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
        # Sınırla (güvenlik)
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

        # Konsol çıktısı (debug — manuel sürüşle aynı format)
        self.get_logger().info(
            f'Sol(Dön): {x1:4d} | Sol(İleri): {y1:4d} || '
            f'Sağ(Yanaş): {x2:4d} | Sağ(Derinlik): {y2:4d} | '
            f'Durum: {self.state}'
        )

    # ── Twist → PWM Dönüşümü ─────────────────────────────────────────────────
    def _twist_to_pwm(self, twist):
        """
        ROS2 Twist mesajını manuel sürüş PWM değerlerine çevirir.
        
        Eşleme (manuel sürüşteki joystick eksenleriyle birebir):
          twist.angular.z  → x1 (Sol Dön)      : -max..+max → 1060..1940
          twist.linear.x   → y1 (Sol İleri)     : -max..+max → 1060..1940
          twist.linear.y   → x2 (Sağ Yanaş)     : -max..+max → 1060..1940
          twist.linear.z   → y2 (Sağ Derinlik)   : -max..+max → 1060..1940
        """
        max_lin = self.p['linear_speed']
        max_ang = self.p['max_angular_z']

        # Angular.z → x1 (dönüş)
        # Joystick'te sol=1060, sağ=1940 idi; angular.z pozitif=sola dönüş
        ang_z_clamped = clamp(twist.angular.z, -max_ang, max_ang)
        x1 = map_value(ang_z_clamped, -max_ang, max_ang, self.PWM_MAX, self.PWM_MIN)

        # Linear.x → y1 (ileri/geri)
        # Joystick'te ileri=1060 (eksen ters), geri=1940
        # ROS'ta linear.x pozitif = ileri
        lin_x_clamped = clamp(twist.linear.x, -max_lin, max_lin)
        # DİKKAT: Joystick Y ekseni ters → ileri=-1 → 1060
        # linear.x pozitif (ileri) → düşük PWM (1060) olmalı
        y1 = map_value(lin_x_clamped, -max_lin, max_lin, self.PWM_MAX, self.PWM_MIN)

        # Linear.y → x2 (yanaşma/strafe) — şu an çizgi takipte kullanılmıyor
        lin_y_clamped = clamp(twist.linear.y, -max_lin, max_lin)
        x2 = map_value(lin_y_clamped, -max_lin, max_lin, self.PWM_MIN, self.PWM_MAX)

        # Linear.z → y2 (derinlik) — şu an çizgi takipte kullanılmıyor
        lin_z_clamped = clamp(twist.linear.z, -max_lin, max_lin)
        y2 = map_value(lin_z_clamped, -max_lin, max_lin, self.PWM_MIN, self.PWM_MAX)

        return x1, y1, x2, y2

    # ── Parametre tanımlamaları ──────────────────────────────────────────────
    def _declare_all_parameters(self):
        defaults = {
            # === Serial port (manuel sürüş ile aynı) ===
            'serial_port':    'COM8',
            'baud_rate':      115200,
            # === Topic'ler ===
            'camera_topic':   '/camera/image_raw',
            'cmd_vel_topic':  '/cmd_vel',
            'depth_topic':    '/depth_sensor',
            # === Hız sınırları ===
            'linear_speed':   0.15,
            'max_angular_z':  0.8,
            # === PID - yatay hata ===
            'pid_kp':         0.003,
            'pid_ki':         0.0001,
            'pid_kd':         0.001,
            'pid_integral_limit': 100.0,
            # === PID - açı düzeltme ===
            'angle_kp':       0.005,
            'angle_kd':       0.001,
            # === Görüntü işleme ===
            'temporal_buffer_size': 5,
            'roi_top_ratio':  0.4,
            'min_contour_area': 500,
            # === HSV eşikleri ===
            'hsv_lower':      [0, 0, 0],
            'hsv_upper':      [180, 80, 60],
            # === Çizgi kayıp toleransı ===
            'max_lost_frames': 30,
            'search_angular_z': 0.3,
            # === Derinlik kontrolü ===
            'target_depth':   1.0,
        }
        for name, val in defaults.items():
            self.declare_parameter(name, val)

    def _get_params(self):
        names = [
            'serial_port', 'baud_rate',
            'camera_topic', 'cmd_vel_topic', 'depth_topic',
            'linear_speed', 'max_angular_z',
            'pid_kp', 'pid_ki', 'pid_kd', 'pid_integral_limit',
            'angle_kp', 'angle_kd',
            'temporal_buffer_size', 'roi_top_ratio', 'min_contour_area',
            'hsv_lower', 'hsv_upper',
            'max_lost_frames', 'search_angular_z', 'target_depth'
        ]
        return {n: self.get_parameter(n).value for n in names}

    # ── Derinlik callback ────────────────────────────────────────────────────
    def _depth_callback(self, msg: Float32):
        self._current_depth = msg.data

    # ── Watchdog ────────────────────────────────────────────────────────────
    def _watchdog_callback(self):
        dt = time.monotonic() - self._last_image_time
        if dt > 2.0:
            self.get_logger().warn(
                f'⚠️  {dt:.1f}s süredir görüntü yok! Motorlar nötre alınıyor.'
            )
            # Tüm kanalları nötre çek (güvenlik)
            self._send_serial_packet(
                self.PWM_NEUTRAL, self.PWM_NEUTRAL,
                self.PWM_NEUTRAL, self.PWM_NEUTRAL
            )
            self._publish_zero_velocity()

    # ── Ana görüntü callback (line_follower.py ile aynı mantık) ──────────────
    def _image_callback(self, msg: Image):
        self._last_image_time = time.monotonic()

        # ROS Image → OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge hatası: {e}')
            return

        h, w = frame.shape[:2]
        image_center_x = w // 2

        # ── Su altı görüntü iyileştirme ──────────────────────────────────
        enhanced = self.img_processor.enhance(frame)

        # ── Çizgi tespiti ────────────────────────────────────────────────
        cx, cy, contour, debug_frame, angle_deg = self.detector.detect(enhanced)

        # ── State Machine (line_follower.py ile birebir aynı) ────────────
        twist = Twist()

        if cx is not None:
            # ✅ Çizgi bulundu
            self.state = self.STATE_FOLLOWING
            self.lost_counter = 0
            self.last_cx = cx

            # Yatay hata
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
            angular_angle   = self.angle_pid.compute(angle_error)

            twist.linear.x  = self.p['linear_speed']
            twist.angular.z = -(angular_lateral + angular_angle)

            # Debug HUD
            cv2.putText(debug_frame,
                        f'State: FOLLOWING | err:{error_lateral:.0f}px | '
                        f'ang_z:{twist.angular.z:.3f}',
                        (10, debug_frame.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        else:
            # ❌ Çizgi bulunamadı
            self.lost_counter += 1

            if self.lost_counter < self.p['max_lost_frames'] // 2:
                self.state = self.STATE_RECOVERING
                direction = np.sign(self.last_error) if self.last_error != 0 else 1.0
                twist.linear.x  = 0.0
                twist.angular.z = direction * self.p['search_angular_z'] * 0.5

            elif self.lost_counter < self.p['max_lost_frames']:
                self.state = self.STATE_SEARCHING
                twist.linear.x  = 0.0
                twist.angular.z = self.p['search_angular_z']
                self.lateral_pid.reset()
                self.angle_pid.reset()

            else:
                self.state = self.STATE_LOST
                twist = Twist()  # sıfır

            cv2.putText(debug_frame,
                        f'State: {self.state} | lost:{self.lost_counter}',
                        (10, debug_frame.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

        # ── Twist → PWM → Serial gönder ─────────────────────────────────
        x1, y1, x2, y2 = self._twist_to_pwm(twist)
        self._send_serial_packet(x1, y1, x2, y2)

        # ── ROS2 cmd_vel de yayınla (debug/kayıt amaçlı) ────────────────
        self.cmd_pub.publish(twist)

        # ── Debug görüntüsü ──────────────────────────────────────────────
        # PWM bilgilerini debug frame'e yaz
        cv2.putText(debug_frame,
                    f'PWM: {x1},{y1},{x2},{y2}',
                    (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)
        except Exception:
            pass

        # ── Durum mesajı ─────────────────────────────────────────────────
        status_msg = String()
        status_msg.data = (
            f'state={self.state},'
            f'cx={cx},'
            f'error={self.last_error:.1f},'
            f'pwm={x1},{y1},{x2},{y2},'
            f'linear_x={twist.linear.x:.3f},'
            f'angular_z={twist.angular.z:.3f}'
        )
        self.status_pub.publish(status_msg)

    def _publish_zero_velocity(self):
        self.cmd_pub.publish(Twist())

    def destroy_node(self):
        """Kapanırken serial portu kapat ve motorları nötre al."""
        # Önce motorları nötre al
        self._send_serial_packet(
            self.PWM_NEUTRAL, self.PWM_NEUTRAL,
            self.PWM_NEUTRAL, self.PWM_NEUTRAL
        )
        # Serial portu kapat
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
        node._send_serial_packet(
            AutonomousDriverNode.PWM_NEUTRAL,
            AutonomousDriverNode.PWM_NEUTRAL,
            AutonomousDriverNode.PWM_NEUTRAL,
            AutonomousDriverNode.PWM_NEUTRAL,
        )
        node._publish_zero_velocity()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
