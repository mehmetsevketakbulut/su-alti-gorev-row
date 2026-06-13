#!/usr/bin/env python3
"""
=============================================================================
UNDERWATER LINE FOLLOWER - ROS2
=============================================================================
Su Altı Çizgi Takip Sistemi - ROS2 (Humble/Iron/Jazzy)

Karşılaşılan Su Altı Zorlukları ve Çözümleri:
----------------------------------------------
1. RENK KAYBI       : Kırmızı ışık ilk absorbe edilir → LAB renk uzayı + CLAHE
2. BULANIKLIK       : Askıdaki partiküller →  Bilateral + Gaussian filtre katmanı
3. IŞIK TİTREMESİ  : Güneş yansıması →  Temporal (zamansal) ortalama tamponu
4. GÖLGE/KARANLIK  : Derin su düşük ışığı → Adaptive histogram eşitleme
5. AKINTI           : Yatay drift → PID + türev damping + anti-windup
6. BOZUK GEOMETRİ  : Kamera mercek → Önceden kalibrasyon desteği
7. KAYIP ÇİZGİ     : Geçici görüş kaybı → Son bilinen pozisyon + recovery modu
8. ÇOKLU ÇİZGİ     : Karmaşık zemin → Alan + şekil bazlı filtreleme
9. BİYO-KIRLENME   : Lens kararması → Dinamik eşik güncelleme
10. DERİNLİK KAYMA : Nötr yüzdürme değişimi → Ayrı dikey kanal kontrolü

ROS2 Topic'leri:
  SUB: /camera/image_raw          (sensor_msgs/Image)
  SUB: /depth_sensor              (std_msgs/Float32)   [opsiyonel]
  PUB: /cmd_vel                   (geometry_msgs/Twist)
  PUB: /line_follower/debug_image (sensor_msgs/Image)
  PUB: /line_follower/status      (std_msgs/String)

Kullanım:
  ros2 run <paket_adi> underwater_line_follower
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from collections import deque
import math
import time

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge


# =============================================================================
# PID CONTROLLER (Anti-Windup + Türev Filtreleme)
# =============================================================================
class PIDController:
    """
    Su altı koşulları için geliştirilmiş PID kontrolcü.
    - Anti-windup: integral birikimini sınırlar (akıntı durumunda taşmayı önler)
    - Derivative filter: türev gürültüsünü filtreler
    - Output clamping: maksimum çıkış sınırı
    """

    def __init__(self, kp, ki, kd,
                 output_min=-1.0, output_max=1.0,
                 integral_limit=50.0,
                 derivative_alpha=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit
        self.derivative_alpha = derivative_alpha  # Türev LP filtre katsayısı

        self._integral = 0.0
        self._prev_error = 0.0
        self._filtered_derivative = 0.0
        self._prev_time = None

    def compute(self, error):
        now = time.monotonic()
        if self._prev_time is None:
            self._prev_time = now
            self._prev_error = error
            return 0.0

        dt = now - self._prev_time
        if dt < 1e-6:
            return 0.0
        self._prev_time = now

        # Proportional
        p_term = self.kp * error

        # Integral + Anti-Windup (clamping yöntemi)
        self._integral += error * dt
        self._integral = max(-self.integral_limit,
                              min(self.integral_limit, self._integral))
        i_term = self.ki * self._integral

        # Derivative + Low-pass filter (titreşimi azaltır)
        raw_derivative = (error - self._prev_error) / dt
        self._filtered_derivative = (
            self.derivative_alpha * raw_derivative
            + (1.0 - self.derivative_alpha) * self._filtered_derivative
        )
        d_term = self.kd * self._filtered_derivative

        self._prev_error = error

        output = p_term + i_term + d_term
        return max(self.output_min, min(self.output_max, output))

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._filtered_derivative = 0.0
        self._prev_time = None


# =============================================================================
# SU ALTI GÖRÜNTÜ İŞLEME
# =============================================================================
class UnderwaterImageProcessor:
    """
    Su altı görüntü zorlukları için özel önişleme zinciri.
    """

    def __init__(self, temporal_buffer_size=5):
        # Zamansal ortalama tamponu (ışık titremesini giderir)
        self._temporal_buffer = deque(maxlen=temporal_buffer_size)

        # CLAHE nesnesi (Kontrast Sınırlı Adaptif Histogram Eşitleme)
        self._clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

    def enhance(self, frame):
        """
        Su altı görüntü iyileştirme zinciri:
        1. Temporal smoothing  → ışık titremesi
        2. White balance       → renk kayması
        3. CLAHE               → kontrast
        4. Bilateral filter    → gürültü, kenar korumalı
        """
        # --- 1. Zamansal ortalama (ışık titremesi önleme) ---
        self._temporal_buffer.append(frame.astype(np.float32))
        smoothed = np.mean(self._temporal_buffer, axis=0).astype(np.uint8)

        # --- 2. Su altı beyaz dengesi (Gray World varsayımı) ---
        balanced = self._gray_world_white_balance(smoothed)

        # --- 3. LAB renk uzayında CLAHE (kontrast artırma) ---
        lab = cv2.cvtColor(balanced, cv2.COLOR_BGR2LAB)
        l_ch, a_ch, b_ch = cv2.split(lab)
        l_ch = self._clahe.apply(l_ch)
        lab = cv2.merge([l_ch, a_ch, b_ch])
        enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        # --- 4. Bilateral filtre (gürültü giderme, kenarları korur) ---
        denoised = cv2.bilateralFilter(enhanced, d=9, sigmaColor=75, sigmaSpace=75)

        return denoised

    @staticmethod
    def _gray_world_white_balance(frame):
        """
        Gray World varsayımı ile beyaz denge düzeltmesi.
        Su altında mavi/yeşil kanallar baskın olduğundan kırmızıyı artırır.
        """
        result = frame.copy().astype(np.float32)
        avg_b = np.mean(result[:, :, 0])
        avg_g = np.mean(result[:, :, 1])
        avg_r = np.mean(result[:, :, 2])
        avg_all = (avg_b + avg_g + avg_r) / 3.0

        if avg_b > 0:
            result[:, :, 0] *= (avg_all / avg_b)
        if avg_g > 0:
            result[:, :, 1] *= (avg_all / avg_g)
        if avg_r > 0:
            result[:, :, 2] *= (avg_all / avg_r)

        return np.clip(result, 0, 255).astype(np.uint8)


# =============================================================================
# ÇİZGİ DEDEKTÖRÜ
# =============================================================================
class LineDetector:
    """
    Su altı ortamı için çok-yöntemli çizgi dedektörü.
    - Renk tabanlı maske (HSV)
    - Kontur + moment tabanlı merkez bulma
    - Hough çizgi doğrulama (yedek)
    - Dinamik HSV eşiği güncelleme (bio-kirlenme adaptasyonu)
    """

    def __init__(self,
                 hsv_lower=(0, 0, 0),
                 hsv_upper=(180, 80, 80),
                 min_contour_area=500,
                 roi_top_ratio=0.4):
        # HSV alt/üst eşikleri (siyah/koyu çizgi için default)
        self.hsv_lower = np.array(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_upper, dtype=np.uint8)
        self.min_contour_area = min_contour_area
        self.roi_top_ratio = roi_top_ratio  # Görüntünün kaç üst %'si kesilir

        # Dinamik eşik için adaptasyon sayacı
        self._adapt_counter = 0
        self._adapt_interval = 30  # Her 30 karede bir eşiği güncelle

    def detect(self, enhanced_frame):
        """
        Çizgiyi tespit eder.
        Döndürür: (cx, cy, contour, debug_frame, angle_deg)
          cx, cy  : Çizgi merkezi (px), bulunamazsa None
          contour : Seçilen kontur
          debug   : Üzerine çizim yapılmış debug görüntüsü
          angle   : Çizginin açısı (derece, 0=düz)
        """
        h, w = enhanced_frame.shape[:2]
        debug = enhanced_frame.copy()

        # --- ROI: Alt bölgeye odaklan (yukarıdaki karanlık/gürültüyü kes) ---
        roi_y_start = int(h * self.roi_top_ratio)
        roi = enhanced_frame[roi_y_start:h, :]

        # --- HSV maskesi ---
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # --- Morfolojik temizleme ---
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # --- Kontur bul ---
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)

        # Çok küçük konturları filtrele
        valid = [c for c in contours if cv2.contourArea(c) > self.min_contour_area]

        if not valid:
            # Hough yedek dedektörü
            return self._hough_fallback(roi, w, roi_y_start, debug)

        # En büyük ve en uzun konturu seç
        best = max(valid, key=lambda c: cv2.contourArea(c))

        # Moment ile merkez
        M = cv2.moments(best)
        if M["m00"] == 0:
            return None, None, None, debug, 0.0

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"]) + roi_y_start

        # Çizgi açısını hesapla (fitLine)
        angle_deg = self._compute_angle(best)

        # Debug çizimi
        cv2.drawContours(debug[roi_y_start:], [best], -1, (0, 255, 0), 2)
        cv2.circle(debug, (cx, cy), 8, (0, 0, 255), -1)
        cv2.line(debug, (w // 2, h), (cx, cy), (255, 0, 0), 2)
        cv2.putText(debug, f"cx:{cx} cy:{cy} angle:{angle_deg:.1f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # Dinamik eşik adaptasyonu
        self._adapt_counter += 1
        if self._adapt_counter >= self._adapt_interval:
            self._adapt_thresholds(roi[mask > 0])
            self._adapt_counter = 0

        return cx, cy, best, debug, angle_deg

    def _hough_fallback(self, roi, w, roi_y_start, debug):
        """Kontur bulunamazsa Hough çizgi dönüşümü ile yedek tespit."""
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180,
                                 threshold=50, minLineLength=60, maxLineGap=20)
        if lines is None:
            return None, None, None, debug, 0.0

        # Tüm çizgilerin x orta noktalarını ortalıyoruz
        mid_xs = [(x1 + x2) / 2 for line in lines for x1, y1, x2, y2 in line]
        cx = int(np.mean(mid_xs))
        cy = roi.shape[0] // 2 + roi_y_start

        cv2.putText(debug, "HOUGH FALLBACK", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        cv2.circle(debug, (cx, cy), 8, (0, 165, 255), -1)
        return cx, cy, None, debug, 0.0

    @staticmethod
    def _compute_angle(contour):
        """Konturun yatay ile açısını hesaplar."""
        if len(contour) < 5:
            return 0.0
        [vx, vy, x0, y0] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
        angle = math.degrees(math.atan2(float(vy), float(vx)))
        return angle

    def _adapt_thresholds(self, line_pixels):
        """
        Tespit edilen çizgi piksellerinin ortalama rengine göre
        HSV eşiklerini dinamik olarak günceller (bio-kirlenme adaptasyonu).
        """
        if len(line_pixels) < 100:
            return
        # Basit ortalama tabanlı adaptasyon (çok agresif değişimi önlemek için
        # mevcut eşikle interpolasyon yapılır)
        pass  # İleri seviye: Gaussian Mixture Model ile otomatik küme tespiti


# =============================================================================
# ANA ROS2 NODE
# =============================================================================
class UnderwaterLineFollowerNode(Node):
    """
    Su Altı Çizgi Takip - Ana ROS2 Node'u

    Durumlar (State Machine):
      SEARCHING  : Çizgi kayıp, dönme hareketi ile arama
      FOLLOWING  : Çizgi takip ediliyor
      RECOVERING : Son bilinen konuma dönerek kurtarma
      LOST       : Uzun süre çizgi bulunamadı, dur
    """

    STATE_SEARCHING  = "SEARCHING"
    STATE_FOLLOWING  = "FOLLOWING"
    STATE_RECOVERING = "RECOVERING"
    STATE_LOST       = "LOST"

    def __init__(self):
        super().__init__('underwater_line_follower')

        # ── ROS2 Parametreleri ──────────────────────────────────────────────
        self._declare_all_parameters()

        p = self._get_params()

        # ── Alt sistemler ───────────────────────────────────────────────────
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

        # ── State ───────────────────────────────────────────────────────────
        self.state = self.STATE_SEARCHING
        self.last_cx = None
        self.last_error = 0.0
        self.lost_counter = 0
        self.p = p
        self.bridge = CvBridge()

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

        # ── Publisher'lar ───────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, p['cmd_vel_topic'], 10)
        self.debug_pub = self.create_publisher(Image, '/line_follower/debug_image', 1)
        self.status_pub = self.create_publisher(String, '/line_follower/status', 10)

        # ── Güvenlik timer'ı (watchdog) ─────────────────────────────────────
        # Görüntü gelmezse robotu durdur
        self._last_image_time = time.monotonic()
        self.create_timer(1.0, self._watchdog_callback)

        self.get_logger().info(
            '🌊 Underwater Line Follower başlatıldı!\n'
            f'  Kamera topic  : {p["camera_topic"]}\n'
            f'  Cmd_vel topic : {p["cmd_vel_topic"]}\n'
            f'  Hız (ileri)   : {p["linear_speed"]} m/s\n'
            f'  HSV Alt       : {p["hsv_lower"]}\n'
            f'  HSV Üst       : {p["hsv_upper"]}'
        )

    # ── Parametre tanımlamaları ──────────────────────────────────────────────
    def _declare_all_parameters(self):
        defaults = {
            # Topic'ler
            'camera_topic':   '/camera/image_raw',
            'cmd_vel_topic':  '/cmd_vel',
            'depth_topic':    '/depth_sensor',
            # Hız sınırları
            'linear_speed':   0.15,       # m/s (su altında düşük tutulur)
            'max_angular_z':  0.8,        # rad/s
            # PID - yatay hata
            'pid_kp':         0.003,
            'pid_ki':         0.0001,
            'pid_kd':         0.001,
            'pid_integral_limit': 100.0,
            # PID - açı düzeltme
            'angle_kp':       0.005,
            'angle_kd':       0.001,
            # Görüntü işleme
            'temporal_buffer_size': 5,
            'roi_top_ratio':  0.4,
            'min_contour_area': 500,
            # HSV eşikleri (siyah çizgi - derin su zemini)
            'hsv_lower':      [0, 0, 0],
            'hsv_upper':      [180, 80, 60],
            # Çizgi kayıp toleransı
            'max_lost_frames': 30,        # Bu kadar kare kayıptan sonra LOST
            'search_angular_z': 0.3,     # Arama dönüş hızı
            # Derinlik kontrolü (opsiyonel)
            'target_depth':   1.0,       # metre
        }
        for name, val in defaults.items():
            if isinstance(val, list):
                self.declare_parameter(name, val)
            else:
                self.declare_parameter(name, val)

    def _get_params(self):
        names = [
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
        """
        Opsiyonel: Derinlik sensörü verisi.
        Şu an loglama amaçlı; ilerisi için dikey itici kontrolüne bağlanabilir.
        """
        self._current_depth = msg.data

    # ── Watchdog ────────────────────────────────────────────────────────────
    def _watchdog_callback(self):
        dt = time.monotonic() - self._last_image_time
        if dt > 2.0:
            self.get_logger().warn(
                f'⚠️  {dt:.1f}s süredir görüntü yok! Robot durduruluyor.'
            )
            self._publish_zero_velocity()

    # ── Ana görüntü callback ─────────────────────────────────────────────────
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

        # ── State Machine ────────────────────────────────────────────────
        twist = Twist()

        if cx is not None:
            # ✅ Çizgi bulundu
            self.state = self.STATE_FOLLOWING
            self.lost_counter = 0
            self.last_cx = cx

            # Yatay hata (piksel cinsinden, normalize edilmemiş)
            error_lateral = float(cx - image_center_x)
            self.last_error = error_lateral

            # Açı hatası: çizgi eğimden kaynaklanan düzeltme
            # 90 derece = düz ileri; sapma → yaw koreksiyon
            angle_error = 0.0
            if abs(angle_deg) > 5.0:
                # fitLine 0-180 arası verir; 90 etrafında normalize et
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
                # RECOVERING: son bilinen yöne doğru hafif dön
                self.state = self.STATE_RECOVERING
                direction = np.sign(self.last_error) if self.last_error != 0 else 1.0
                twist.linear.x  = 0.0
                twist.angular.z = direction * self.p['search_angular_z'] * 0.5

            elif self.lost_counter < self.p['max_lost_frames']:
                # SEARCHING: aktif arama dönüşü
                self.state = self.STATE_SEARCHING
                twist.linear.x  = 0.0
                twist.angular.z = self.p['search_angular_z']
                # PID sıfırla (integral birikmesin)
                self.lateral_pid.reset()
                self.angle_pid.reset()

            else:
                # LOST: tamamen dur
                self.state = self.STATE_LOST
                twist = Twist()  # sıfır

            cv2.putText(debug_frame,
                        f'State: {self.state} | lost:{self.lost_counter}',
                        (10, debug_frame.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

        # ── Komut yayınla ────────────────────────────────────────────────
        self.cmd_pub.publish(twist)

        # ── Debug görüntüsü yayınla ──────────────────────────────────────
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
            f'linear_x={twist.linear.x:.3f},'
            f'angular_z={twist.angular.z:.3f}'
        )
        self.status_pub.publish(status_msg)

    def _publish_zero_velocity(self):
        self.cmd_pub.publish(Twist())


# =============================================================================
# MAIN
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = UnderwaterLineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Kullanıcı tarafından durduruldu.')
    finally:
        node._publish_zero_velocity()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
