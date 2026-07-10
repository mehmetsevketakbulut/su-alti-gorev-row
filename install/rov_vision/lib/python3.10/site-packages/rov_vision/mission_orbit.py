#!/usr/bin/env python3
"""
=============================================================================
MISSION ORBIT — Şamandıra Yörünge Görev Kontrolcüsü (OpenCV + PID)
=============================================================================
Teknofest İnsansız Su Altı Sistemleri Yarışması — Antigravity Takımı

Görev Akışı:
  INIT → GOTO_WAYPOINT → VISUAL_SEARCH → APPROACH → ORBITING → GOTO_CENTER → DONE

Özellikler:
  • Kontur tabanlı gelişmiş kırmızı şamandıra tespiti (çift HSV aralığı)
  • PID tabanlı Visual Servoing (şamandırayı kamera merkezine kilitleme)
  • Orbit sırasında mesafe + yön PID ile kararlı çember yörüngesi
  • Tüm parametreler üst kısımda kolayca kalibre edilebilir
  • DVL olmadan Dead Reckoning + IMU heading hold uyumlu

Yayınlar:
  /cmd_vel (geometry_msgs/Twist)  — Alt seviye sürücüye hareket komutu
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
import time
import math


class OrbitMissionNode(Node):
    """
    Şamandıra yörünge görevini yöneten state machine tabanlı ROS 2 node'u.

    Dead Reckoning (sabit hız × süre) ile waypoint'e gider, OpenCV ile
    şamandırayı bulur, PID ile kilitlenir, yaklaşır ve şamandıra etrafında
    kavisli yörünge çizer. Sonunda merkez bölgeye park eder.
    """

    # ╔═══════════════════════════════════════════════════════════════════════╗
    # ║                    KALİBRASYON PARAMETRELERİ                        ║
    # ║  Havuz testlerinde bu değerleri değiştirin, kodun geri kalanına      ║
    # ║  dokunmanıza gerek yok.                                             ║
    # ╚═══════════════════════════════════════════════════════════════════════╝

    # ── HSV Renk Eşikleri (Kırmızı şamandıra) ───────────────────────────────
    # Kırmızı renk HSV uzayında 0° ve 180° civarında iki bölgeye ayrılır.
    # Bu yüzden iki ayrı aralık kullanıyoruz ve birleştiriyoruz (OR maskesi).
    HSV_LOWER_1 = np.array([0,   120,  70])    # Alt kırmızı bölge — alt sınır
    HSV_UPPER_1 = np.array([10,  255, 255])    # Alt kırmızı bölge — üst sınır
    HSV_LOWER_2 = np.array([170, 120,  70])    # Üst kırmızı bölge — alt sınır
    HSV_UPPER_2 = np.array([180, 255, 255])    # Üst kırmızı bölge — üst sınır

    # ── Morfoloji / Gürültü Filtreleme ───────────────────────────────────────
    MORPH_KERNEL_SIZE = 5         # Morfolojik işlem kernel boyutu (piksel)
    MIN_CONTOUR_AREA  = 800       # Bu alandan küçük konturları gürültü say (piksel²)

    # ── PID Katsayıları — Yatay Kilitleme (Visual Servoing) ──────────────────
    # Hata = (şamandıra_cx - görüntü_merkez_x), normalleştirilmiş [-1, +1]
    YAW_KP = 0.8     # Oransal: Ne kadar hızlı düzeltme yapılsın
    YAW_KI = 0.02    # İntegral: Kalıcı küçük hataları telafi eder
    YAW_KD = 0.15    # Türev:   Aşırı salınımı (overshoot) önler

    # ── PID Katsayıları — Mesafe Kontrolü (Orbit sırasında) ──────────────────
    # Hata = (hedef_alan - mevcut_alan) / hedef_alan, normalleştirilmiş
    DIST_KP = 0.5    # Oransal: Mesafe düzeltme agresifliği
    DIST_KI = 0.01   # İntegral: Sabit mesafe sapmasını giderir
    DIST_KD = 0.10   # Türev:   Ani mesafe değişimlerini yumuşatır

    # ── PID Limitleri ────────────────────────────────────────────────────────
    PID_INTEGRAL_LIMIT = 0.5      # İntegral birikiminin azami mutlak değeri
    MAX_ANGULAR_Z      = 0.6      # Azami dönüş hızı (rad/s)
    MAX_LINEAR_X       = 0.5      # Azami ileri/geri hız (m/s)
    MAX_LINEAR_Y       = 0.5      # Azami yanal hız (m/s)

    # ── Durum Süreleri (Dead Reckoning) ──────────────────────────────────────
    INIT_WAIT_DURATION     = 5.0   # Başlangıç bekleme (saniye) — sensör stabilizasyonu
    BLIND_DRIVE_DURATION   = 16.0  # Şamandıraya doğru kör ilerleme süresi (saniye)
    BLIND_DRIVE_SPEED      = 0.5   # Kör ilerleme sırasında ileri hız (m/s)
    SEARCH_YAW_SPEED       = 0.25  # Arama döndürme hızı (rad/s)
    SEARCH_TIMEOUT         = 30.0  # Aramada maks bekleme (saniye) — zaman aşımı güvenliği
    APPROACH_SPEED         = 0.3   # Yaklaşma ileri hızı (m/s)
    ORBIT_DURATION         = 25.0  # Yörünge turu süresi (saniye) — 360° için kalibre et
    ORBIT_LATERAL_SPEED    = 0.35  # Yörünge sırasında yanal hız — linear.y (m/s)
    GOTO_CENTER_SPEED      = 0.5   # Merkeze gitme hızı (m/s)
    GOTO_CENTER_DURATION   = 6.0   # Merkeze gitme süresi (saniye)

    # ── Yaklaşma / Orbit Hedef Değerleri ─────────────────────────────────────
    TARGET_AREA_RATIO    = 0.08    # Hedef kontur alan oranı (kontur_alanı / toplam_piksel)
                                   # Şamandıra bu kadar büyük görünene kadar yaklaş
    APPROACH_AREA_TOLERANCE = 0.02 # Hedef alana ne kadar yaklaşılırsa "yeterli" sayılsın
    LOCK_ERROR_THRESHOLD    = 0.05 # Yatay hata bu değerin altındaysa "kilitlendi" say

    # ── Kamera Ayarları ──────────────────────────────────────────────────────
    CAMERA_TOPIC = '/camera/image_raw'  # Kamera ROS topic'i

    # ╔═══════════════════════════════════════════════════════════════════════╗
    # ║                         NODE BAŞLATMA                               ║
    # ╚═══════════════════════════════════════════════════════════════════════╝

    def __init__(self):
        super().__init__('orbit_mission_node')

        # ── Görev Durumları ──────────────────────────────────────────────────
        self.STATE_INIT           = "INIT"
        self.STATE_GOTO_WAYPOINT  = "GOTO_WAYPOINT"
        self.STATE_VISUAL_SEARCH  = "VISUAL_SEARCH"
        self.STATE_APPROACH       = "APPROACH"       # YENİ: Yaklaşma durumu
        self.STATE_ORBITING       = "ORBITING"
        self.STATE_GOTO_CENTER    = "GOTO_CENTER"
        self.STATE_DONE           = "DONE"

        self.current_state = self.STATE_INIT

        # ── Zamanlayıcılar ───────────────────────────────────────────────────
        self.state_start_time = time.time()

        # ── OpenCV Tespit Sonuçları ──────────────────────────────────────────
        self.target_detected  = False     # Şamandıra görüldü mü?
        self.target_cx        = 0.0       # Şamandıra merkezi — x (piksel)
        self.target_cy        = 0.0       # Şamandıra merkezi — y (piksel)
        self.target_area      = 0.0       # Şamandıra kontur alanı (piksel²)
        self.image_width      = 640       # Görüntü genişliği (ilk frame'de güncellenir)
        self.image_height     = 480       # Görüntü yüksekliği

        # ── PID İç Durumları — Yaw (Yatay Kilitleme) ────────────────────────
        self.yaw_error_integral  = 0.0
        self.yaw_error_prev      = 0.0
        self.yaw_last_time       = time.time()

        # ── PID İç Durumları — Mesafe (İleri/Geri Düzeltme) ─────────────────
        self.dist_error_integral = 0.0
        self.dist_error_prev     = 0.0
        self.dist_last_time      = time.time()

        # ── Orbit Açı Takibi ─────────────────────────────────────────────────
        self.orbit_accumulated_yaw = 0.0  # Yörüngede toplam döndürülen açı (derece)

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
            "🚀 Yörünge Görev Kontrolcüsü (Gelişmiş OpenCV + PID) Başladı!\n"
            f"   HSV Aralık 1: {self.HSV_LOWER_1} — {self.HSV_UPPER_1}\n"
            f"   HSV Aralık 2: {self.HSV_LOWER_2} — {self.HSV_UPPER_2}\n"
            f"   Min Kontur Alanı: {self.MIN_CONTOUR_AREA} px²\n"
            f"   Yaw PID: Kp={self.YAW_KP}, Ki={self.YAW_KI}, Kd={self.YAW_KD}\n"
            f"   Dist PID: Kp={self.DIST_KP}, Ki={self.DIST_KI}, Kd={self.DIST_KD}\n"
            f"   Orbit Süresi: {self.ORBIT_DURATION}s | Yanal Hız: {self.ORBIT_LATERAL_SPEED} m/s"
        )

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

        # PID birikimlerini sıfırla (yeni duruma temiz başla)
        self._reset_yaw_pid()
        self._reset_dist_pid()

        # Orbit sayacını sıfırla
        if new_state == self.STATE_ORBITING:
            self.orbit_accumulated_yaw = 0.0

    # ╔═══════════════════════════════════════════════════════════════════════╗
    # ║                    PID KONTROL FONKSİYONLARI                        ║
    # ╚═══════════════════════════════════════════════════════════════════════╝

    def _reset_yaw_pid(self):
        """Yaw PID iç durumlarını sıfırlar."""
        self.yaw_error_integral = 0.0
        self.yaw_error_prev = 0.0
        self.yaw_last_time = time.time()

    def _reset_dist_pid(self):
        """Mesafe PID iç durumlarını sıfırlar."""
        self.dist_error_integral = 0.0
        self.dist_error_prev = 0.0
        self.dist_last_time = time.time()

    def _compute_pid(self, error, kp, ki, kd,
                     integral_ref, prev_error_ref, last_time_ref):
        """
        Genel amaçlı PID hesaplayıcısı.

        Args:
            error:          Mevcut hata değeri
            kp, ki, kd:    PID katsayıları
            integral_ref:   Mevcut integral birikimi (güncellenir)
            prev_error_ref: Bir önceki hata değeri (güncellenir)
            last_time_ref:  Son hesaplama zamanı (güncellenir)

        Returns:
            (output, new_integral, new_prev_error, new_time) tuple'ı
        """
        now = time.time()
        dt = now - last_time_ref
        if dt <= 0.0 or dt > 1.0:
            # İlk çağrı veya çok uzun aralık — sadece P terimi uygula
            dt = 0.05

        # Proportional
        p_term = kp * error

        # Integral (anti-windup clamping ile)
        new_integral = integral_ref + error * dt
        new_integral = max(-self.PID_INTEGRAL_LIMIT,
                           min(self.PID_INTEGRAL_LIMIT, new_integral))
        i_term = ki * new_integral

        # Derivative
        d_term = kd * (error - prev_error_ref) / dt

        output = p_term + i_term + d_term
        return output, new_integral, error, now

    def compute_yaw_pid(self, error):
        """
        Yaw (dönüş) PID hesaplar.

        Args:
            error: Normalleştirilmiş yatay hata [-1, +1]
                   Negatif = hedef solda, Pozitif = hedef sağda

        Returns:
            angular_z komutu (rad/s), [-MAX_ANGULAR_Z, +MAX_ANGULAR_Z] aralığında
        """
        output, self.yaw_error_integral, self.yaw_error_prev, self.yaw_last_time = \
            self._compute_pid(
                error,
                self.YAW_KP, self.YAW_KI, self.YAW_KD,
                self.yaw_error_integral,
                self.yaw_error_prev,
                self.yaw_last_time
            )
        # Çıkışı sınırla
        return max(-self.MAX_ANGULAR_Z, min(self.MAX_ANGULAR_Z, output))

    def compute_dist_pid(self, error):
        """
        Mesafe PID hesaplar.

        Args:
            error: Normalleştirilmiş mesafe hatası [-1, +1]
                   Negatif = çok yakın (geri git), Pozitif = çok uzak (ileri git)

        Returns:
            linear_x komutu (m/s), [-MAX_LINEAR_X, +MAX_LINEAR_X] aralığında
        """
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

        İşlem adımları:
          1. BGR → HSV dönüşümü
          2. Çift aralık kırmızı maske (0°-10° ve 170°-180°)
          3. Morfolojik gürültü temizleme (open + close)
          4. Kontur bulma → En büyük konturu seç
          5. Alan filtresi → Gürültü eleme
          6. Moments ile merkez noktası (cx, cy) hesaplama
          7. Debug görüntüsü yayınlama

        Sadece VISUAL_SEARCH, APPROACH ve ORBITING durumlarında aktiftir.
        Diğer durumlarda CPU yükünü önlemek için erken çıkar.
        """
        # Sadece görsel kontrol gereken durumlarda çalış
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
            # Kırmızı renk HSV'de 0° ve 180° etrafında iki bölgeye ayrılır.
            # Her iki bölgeyi de yakalayıp OR ile birleştiriyoruz.
            mask1 = cv2.inRange(hsv, self.HSV_LOWER_1, self.HSV_UPPER_1)
            mask2 = cv2.inRange(hsv, self.HSV_LOWER_2, self.HSV_UPPER_2)
            mask = cv2.bitwise_or(mask1, mask2)

            # ── 4. Morfolojik Gürültü Temizleme ─────────────────────────────
            # Open (erozyonu + dilatasyon): Küçük gürültü noktalarını siler
            # Close (dilatasyon + erozyon): Kontur içindeki boşlukları doldurur
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
                # En büyük konturu bul
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)

                if area >= self.MIN_CONTOUR_AREA:
                    # ── 7. Moments ile Merkez Hesaplama ──────────────────────
                    M = cv2.moments(largest_contour)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        # Sonuçları kaydet
                        self.target_detected = True
                        self.target_cx   = float(cx)
                        self.target_cy   = float(cy)
                        self.target_area = float(area)

                        # ── Debug Çizimi ─────────────────────────────────────
                        # Kontur çizgisi (yeşil)
                        cv2.drawContours(debug_frame, [largest_contour], -1,
                                         (0, 255, 0), 2)
                        # Merkez noktası (kırmızı daire)
                        cv2.circle(debug_frame, (cx, cy), 8, (0, 0, 255), -1)
                        # Dikey merkezleme çizgisi (kameranın ortası)
                        center_x = self.image_width // 2
                        cv2.line(debug_frame, (center_x, 0),
                                 (center_x, self.image_height),
                                 (255, 255, 0), 1)
                        # Hedeften merkeze hata çizgisi (mavi)
                        cv2.line(debug_frame, (cx, cy),
                                 (center_x, cy), (255, 0, 0), 2)

                        # Bilgi metinleri
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
        Twist komutu üretir ve /cmd_vel'e yayınlar.

        Durum Geçiş Diyagramı:
          INIT ─(5s)→ GOTO_WAYPOINT ─(süre)→ VISUAL_SEARCH ─(hedef kilitli)→
          APPROACH ─(hedef yeterince büyük)→ ORBITING ─(süre/tur)→
          GOTO_CENTER ─(süre)→ DONE
        """
        twist = Twist()
        elapsed = time.time() - self.state_start_time

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # DURUM: INIT — Başlatma ve Sensör Stabilizasyonu
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        if self.current_state == self.STATE_INIT:
            # Araç yerinde bekler, IMU/derinlik sensörü stabilize olur
            if elapsed > self.INIT_WAIT_DURATION:
                self.change_state(self.STATE_GOTO_WAYPOINT)

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # DURUM: GOTO_WAYPOINT — Kör İleri Sürüş (Dead Reckoning)
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        elif self.current_state == self.STATE_GOTO_WAYPOINT:
            # Sabit hız × süre ile şamandıra bölgesine ilerle
            # Alt seviye autonomous_driver IMU heading hold sağlar
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
                # Şamandıra görüldü! PID ile kamera merkezine kilitle.
                center_x = self.image_width / 2.0
                # Hatayı normalleştir: [-1, +1] aralığına çek
                # Negatif = hedef solda → sola dön (pozitif angular.z)
                # Pozitif = hedef sağda → sağa dön (negatif angular.z)
                error = (self.target_cx - center_x) / (self.image_width / 2.0)

                # PID ile dönüş hızını hesapla
                angular_z = self.compute_yaw_pid(error)

                # Hata yeterince küçükse → şamandıra kilitlendi, yaklaşmaya başla
                if abs(error) < self.LOCK_ERROR_THRESHOLD:
                    self.get_logger().info(
                        f"🎯 HEDEF KİLİTLENDİ! Hata: {error:.3f}. "
                        f"Yaklaşma başlatılıyor."
                    )
                    self.change_state(self.STATE_APPROACH)
                else:
                    # Henüz kilitlenmedi — PID ile düzeltmeye devam
                    twist.angular.z = -angular_z  # Hatanın tersine dön
                    self.get_logger().info(
                        f"🔍 Arama: Hedef görüldü | "
                        f"Hata: {error:.3f} | "
                        f"Dönüş: {twist.angular.z:.3f} rad/s",
                        throttle_duration_sec=0.5
                    )
            else:
                # Şamandıra görünmüyor — yavaşça dön ve ara
                twist.angular.z = self.SEARCH_YAW_SPEED

                # Zaman aşımı kontrolü
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
                # ── Yaw PID — Şamandırayı kamera merkezinde tut ──────────────
                center_x = self.image_width / 2.0
                yaw_error = (self.target_cx - center_x) / (self.image_width / 2.0)
                angular_z = self.compute_yaw_pid(yaw_error)
                twist.angular.z = -angular_z

                # ── Mesafe kontrolü — Alan oranına göre ileri/dur ────────────
                total_pixels = self.image_width * self.image_height
                current_ratio = self.target_area / total_pixels
                dist_error = (self.TARGET_AREA_RATIO - current_ratio) / self.TARGET_AREA_RATIO

                if dist_error > self.APPROACH_AREA_TOLERANCE:
                    # Henüz yeterince yakın değil — ileri git
                    forward_speed = self.compute_dist_pid(dist_error)
                    twist.linear.x = max(0.0, forward_speed)  # Sadece ileri
                    self.get_logger().info(
                        f"🏊 Yaklaşma: Alan oranı={current_ratio:.4f} "
                        f"Hedef={self.TARGET_AREA_RATIO:.4f} "
                        f"Hız={twist.linear.x:.3f} m/s",
                        throttle_duration_sec=0.5
                    )
                else:
                    # Yeterince yaklaştık — yörüngeye geç!
                    self.get_logger().info(
                        f"✅ Yaklaşma tamamlandı! Alan oranı: {current_ratio:.4f}. "
                        f"Yörünge başlatılıyor."
                    )
                    self.change_state(self.STATE_ORBITING)
            else:
                # Yaklaşma sırasında hedef kayboldu — aramaya geri dön
                self.get_logger().warn(
                    "⚠️ Yaklaşma sırasında hedef kayboldu! Aramaya dönülüyor."
                )
                self.change_state(self.STATE_VISUAL_SEARCH)

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # DURUM: ORBITING — Yörünge Turu (Yengeç Hareketi + PID)
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        elif self.current_state == self.STATE_ORBITING:
            if self.target_detected:
                # ── 1. Yaw PID — Şamandırayı her zaman kamera merkezinde tut ─
                center_x = self.image_width / 2.0
                yaw_error = (self.target_cx - center_x) / (self.image_width / 2.0)
                angular_z = self.compute_yaw_pid(yaw_error)
                twist.angular.z = -angular_z

                # ── 2. Mesafe PID — Sabit mesafeyi koru ──────────────────────
                # Şamandıra çok büyük görünüyorsa → uzaklaş (geri git)
                # Çok küçük görünüyorsa → yaklaş (ileri git)
                total_pixels = self.image_width * self.image_height
                current_ratio = self.target_area / total_pixels
                dist_error = (self.TARGET_AREA_RATIO - current_ratio) / self.TARGET_AREA_RATIO
                forward_correction = self.compute_dist_pid(dist_error)
                twist.linear.x = forward_correction

                # ── 3. Yanal hız — Yengeç hareketi (daima sağa kayarak çember) ─
                twist.linear.y = self.ORBIT_LATERAL_SPEED

                # ── Orbit tamamlanma kontrolü (süre bazlı) ──────────────────
                self.get_logger().info(
                    f"🔄 Yörünge: {elapsed:.1f}/{self.ORBIT_DURATION:.0f}s | "
                    f"Yaw hata: {yaw_error:.3f} | "
                    f"Alan oranı: {current_ratio:.4f} | "
                    f"İleri düzeltme: {twist.linear.x:.3f}",
                    throttle_duration_sec=1.0
                )

            else:
                # Şamandıra görünmüyor — yörüngeye devam et ama düzeltme yapma
                # (kısa süreli kayıplarda yörüngeyi bozmamak için)
                twist.linear.y = self.ORBIT_LATERAL_SPEED
                twist.angular.z = self.SEARCH_YAW_SPEED * 0.5
                self.get_logger().warn(
                    "⚠️ Yörünge sırasında hedef geçici kayıp! "
                    "Kör yörünge devam ediyor.",
                    throttle_duration_sec=1.0
                )

            # Süre bazlı orbit sonlandırma
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
            # Tüm hızlar sıfır (Twist varsayılanı)
            if elapsed < 1.0:
                self.get_logger().info(
                    "✅ GÖREV TAMAMLANDI — Motorlar nötr.",
                    throttle_duration_sec=5.0
                )

        # Komutu yayınla
        self.cmd_pub.publish(twist)


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
        node.cmd_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()