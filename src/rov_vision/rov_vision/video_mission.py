"""
Teknofest İnsansız Su Altı Sistemleri - Video Kanıtı Görevi
Dead Reckoning (Kör Sürüş) Tabanlı State Machine

── MOTOR KONTROLÜ ──
AnaROV Yer İstasyonu (Base Station) protokolü kullanılır:
  - Serial port : COM8 (parametre ile değiştirilebilir)
  - Baud rate   : 115200
  - Paket formatı: "A,m1,m2,m3,m4,m5,m6,btn,kp,kd\n"
  - Değerler    : -100 ile +100 arası yüzdelik dilimler
  - Kanal eşlemeleri:
      m1 → Ön Sağ (Yatay)
      m2 → Ön Sol (Yatay)
      m3 → Arka Sağ (Yatay)
      m4 → Arka Sol (Yatay)
      m5 → Dikey Sol
      m6 → Dikey Sağ

── PARKUR SIRASI ──
  DURUM 0 - WAIT              : Başlangıç alanında bekle (5s)
  DURUM 1 - DIVE              : Su altına dalış (3-4s agresif batma itkisi)
  DURUM 2 - FORWARD_1         : Düz ileri git (derinlik koruma aktif)
  DURUM 3 - TURN_1            : Sağa 90° dön
  DURUM 4 - FORWARD_2         : Düz ileri git
  DURUM 5 - CIRCLE            : 360° daire çiz (linear.x + angular.z)
  DURUM 5.5 - TURN_AFTER_CIRCLE : Daire sonrası 90° sağa dön
  DURUM 6 - FORWARD_3         : Düz ileri git
  DURUM 7 - TURN_2            : Sağa 90° dön
  DURUM 8 - FORWARD_4         : Düz ileri git → başlangıca dön
  DURUM 9 - SURFACE           : Yüzeye çıkış (yukarı itki)

── DERİNLİK KONTROLÜ ──
  Araç pozitif batmaz (yüzer). Görev boyunca sürekli aşağı itki uygulanır.
  DIVE durumunda agresif dalış, diğer durumlarda hafif derinlik koruma.
  Görev bitince SURFACE durumunda yukarı itki ile yüzeye çıkış.

Kamera / Sensör (IMU) abonesi YOKTUR.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time


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
#  VIDEO MISSION NODE
# ══════════════════════════════════════════════════════════════════
class VideoMissionNode(Node):

    # ── Durum sabitleri ──────────────────────────────────────────
    STATE_WAIT              = 'WAIT'              # Durum 0: Bekleme
    STATE_DIVE              = 'DIVE'              # Durum 1: Dalış
    STATE_FORWARD_1         = 'FORWARD_1'         # Durum 2
    STATE_TURN_1            = 'TURN_1'            # Durum 3
    STATE_FORWARD_2         = 'FORWARD_2'         # Durum 4
    STATE_CIRCLE            = 'CIRCLE'            # Durum 5
    STATE_TURN_AFTER_CIRCLE = 'TURN_AFTER_CIRCLE' # Durum 5.5
    STATE_FORWARD_3         = 'FORWARD_3'         # Durum 6
    STATE_TURN_2            = 'TURN_2'            # Durum 7
    STATE_FORWARD_4         = 'FORWARD_4'         # Durum 8
    STATE_SURFACE           = 'SURFACE'           # Durum 9: Yüzeye çıkış
    STATE_FINISHED          = 'FINISHED'

    # ── Haberleşme Sabitleri ─────────────────────────────────────
    # Format: A,M1,M2,M3,M4,M5,M6,KapatmaTusu,Kp,Kd
    # Değerler: -100 ile +100 arası yüzdelik dilimler
    KP_DEFAULT = 150  # 1.5 * 100
    KD_DEFAULT = 25   # 0.25 * 100

    def __init__(self):
        super().__init__('video_mission_node')

        # ==========================================================
        #  ROS 2 PARAMETRELERİ
        # ==========================================================

        # ── Serial port (autonomous_driver.py ile aynı) ───────────
        self.declare_parameter('serial_port', 'COM8')
        self.declare_parameter('baud_rate',   115200)

        # ── Süre parametreleri (saniye) ───────────────────────────
        self.declare_parameter('duration_wait',      5.0)
        self.declare_parameter('duration_dive',      3.5)   # Dalış süresi
        self.declare_parameter('duration_forward',  15.0)
        self.declare_parameter('duration_turn_90',   2.5)
        self.declare_parameter('duration_circle',   12.0)
        self.declare_parameter('duration_surface',   5.0)   # Yüzeye çıkış süresi

        # ── Hız parametreleri (Twist -1.0 ... +1.0 arası) ────────
        self.declare_parameter('speed_forward',      0.4)
        self.declare_parameter('speed_turn',         0.5)
        self.declare_parameter('circle_linear_x',    0.3)
        self.declare_parameter('circle_angular_z',   0.52)

        # ── Derinlik kontrol parametreleri ─────────────────────────
        self.declare_parameter('dive_speed',         0.5)   # Agresif dalış hızı (DIVE durumu)
        self.declare_parameter('hold_depth_speed',   0.2)   # Görev boyunca derinlik koruma hızı
        self.declare_parameter('surface_speed',      0.4)   # Yüzeye çıkış hızı
        self.declare_parameter('invert_vertical',    False)  # Dikey yön ters ise True yap

        # ── PWM dönüşümü için max aralık (autonomous_driver ile aynı)
        self.declare_parameter('max_linear',         1.0)
        self.declare_parameter('max_angular',        1.0)
        self.declare_parameter('max_vertical',       1.0)
        
        self.declare_parameter('power_limit_percent', 100.0) # Maksimum güç sınırı %100

        # ── Timer frekansı ────────────────────────────────────────
        self.declare_parameter('loop_rate_hz',      20.0)

        # ── Parametreleri oku ─────────────────────────────────────
        self._read_parameters()

        # ── Serial port aç ───────────────────────────────────────
        self.ser = None
        self._open_serial()

        # ── Durum makinesi ────────────────────────────────────────
        self.current_state = self.STATE_WAIT
        self.state_start_time = time.time()

        # ── Publisher (debug/kayıt amaçlı) ────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Kontrol döngüsü ──────────────────────────────────────
        loop_period = 1.0 / self.loop_rate_hz
        self.timer = self.create_timer(loop_period, self._mission_loop)

        self.get_logger().info('=' * 60)
        self.get_logger().info('🚀 VIDEO GÖREV KONTROLCÜSÜ HAZIR!')
        self.get_logger().info(f'   Serial port         : {self.serial_port} @ {self.baud_rate}')
        self.get_logger().info(f'   Haberleşme Formatı  : AnaROV Base Station (A,m1,m2... formatı)')
        self.get_logger().info(f'   ─── Süre Ayarları ───')
        self.get_logger().info(f'   Bekleme süresi      : {self.duration_wait:.1f} s')
        self.get_logger().info(f'   Dalış süresi        : {self.duration_dive:.1f} s')
        self.get_logger().info(f'   Düz gitme süresi    : {self.duration_forward:.1f} s')
        self.get_logger().info(f'   90° dönüş süresi    : {self.duration_turn_90:.2f} s')
        self.get_logger().info(f'   Daire süresi        : {self.duration_circle:.1f} s')
        self.get_logger().info(f'   Yüzeye çıkış süresi : {self.duration_surface:.1f} s')
        self.get_logger().info(f'   ─── Hız Ayarları ───')
        self.get_logger().info(f'   Düz gitme hızı      : {self.speed_forward:.2f}')
        self.get_logger().info(f'   Dönüş hızı          : {self.speed_turn:.2f}')
        self.get_logger().info(f'   Daire linear.x      : {self.circle_linear_x:.2f}')
        self.get_logger().info(f'   Daire angular.z     : {self.circle_angular_z:.2f}')
        self.get_logger().info(f'   ─── Derinlik Ayarları ───')
        self.get_logger().info(f'   Dalış hızı (agresif): {self.dive_speed:.2f}')
        self.get_logger().info(f'   Derinlik koruma     : {self.hold_depth_speed:.2f}')
        self.get_logger().info(f'   Yüzeye çıkış hızı   : {self.surface_speed:.2f}')
        self.get_logger().info(f'   Dikey ters          : {self.invert_vertical}')
        self.get_logger().info('=' * 60)

    # ══════════════════════════════════════════════════════════════
    #  PARAMETRE OKUMA
    # ══════════════════════════════════════════════════════════════
    def _read_parameters(self):
        self.serial_port      = self.get_parameter('serial_port').value
        self.baud_rate        = self.get_parameter('baud_rate').value

        self.duration_wait    = self.get_parameter('duration_wait').value
        self.duration_dive    = self.get_parameter('duration_dive').value
        self.duration_forward = self.get_parameter('duration_forward').value
        self.duration_turn_90 = self.get_parameter('duration_turn_90').value
        self.duration_circle  = self.get_parameter('duration_circle').value
        self.duration_surface = self.get_parameter('duration_surface').value

        self.speed_forward    = self.get_parameter('speed_forward').value
        self.speed_turn       = self.get_parameter('speed_turn').value
        self.circle_linear_x  = self.get_parameter('circle_linear_x').value
        self.circle_angular_z = self.get_parameter('circle_angular_z').value

        self.dive_speed       = self.get_parameter('dive_speed').value
        self.hold_depth_speed = self.get_parameter('hold_depth_speed').value
        self.surface_speed    = self.get_parameter('surface_speed').value
        self.invert_vertical  = self.get_parameter('invert_vertical').value

        self.max_linear       = self.get_parameter('max_linear').value
        self.max_angular      = self.get_parameter('max_angular').value
        self.max_vertical     = self.get_parameter('max_vertical').value
        self.power_limit_percent = self.get_parameter('power_limit_percent').value
        self.loop_rate_hz     = self.get_parameter('loop_rate_hz').value

    # ══════════════════════════════════════════════════════════════
    #  SERIAL PORT (autonomous_driver.py ile birebir aynı)
    # ══════════════════════════════════════════════════════════════
    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f'✅ Serial bağlantı kuruldu: {self.serial_port}')
        except Exception as e:
            self.get_logger().error(
                f'❌ Serial bağlantı BAŞARISIZ: {e}\n'
                f'  → Araç hareket ETMEYECEK! Portu kontrol et.'
            )
            self.ser = None

    def _send_serial(self, m1, m2, m3, m4, m5, m6):
        """
        Base Station Arduino'ya (CAN Transmitter) komut gönderir.
        Format: "A,M1,M2,M3,M4,M5,M6,btn_kapat,kp,kd\n"
        Değerler -100 ile +100 arasındadır.
        """
        btn_kapat = 0
        paket = f"A,{m1},{m2},{m3},{m4},{m5},{m6},{btn_kapat},{self.KP_DEFAULT},{self.KD_DEFAULT}\n"

        if self.ser and self.ser.is_open:
            try:
                self.ser.write(paket.encode('utf-8'))
            except Exception as e:
                self.get_logger().warn(f'Serial yazma hatası: {e}')

    # ══════════════════════════════════════════════════════════════
    #  TWIST → YÜZDE (%) DÖNÜŞÜMÜ (MIXER)
    # ══════════════════════════════════════════════════════════════
    def _twist_to_percentages(self, twist):
        """
        Manuel sürüşteki 'calculate_thruster_mix' fonksiyonunun aynısıdır.
        Twist mesajını alır, -100 ile +100 arası 6 motor değerine çevirir.
        """
        # ROS 2 -> Joystick Eksen Eşleşmesi
        # twist.linear.x  (İleri) -> fwd_rev
        # twist.linear.y  (Sol)   -> -strain_lr (Joystick sağa pozitif)
        # twist.linear.z  (Yukarı)-> dive_ud
        # twist.angular.z (Sol)   -> -yaw_lr    (Joystick sağa pozitif)
        
        fwd_rev   = twist.linear.x
        strain_lr = -twist.linear.y
        dive_ud   = twist.linear.z
        yaw_lr    = -twist.angular.z

        if self.invert_vertical:
            dive_ud = -dive_ud

        # Yatay Motorlar (Vektörel Mikser)
        m_fr = fwd_rev - strain_lr - yaw_lr
        m_fl = fwd_rev + strain_lr + yaw_lr
        m_rr = fwd_rev + strain_lr - yaw_lr
        m_rl = fwd_rev - strain_lr + yaw_lr
        
        # Dikey Motorlar
        m_vf = dive_ud
        m_vr = dive_ud
        
        thrusters = [m_fr, m_fl, m_rr, m_rl, m_vf, m_vr]
        scaled_thrusters = []
        
        for val in thrusters:
            val = max(-1.0, min(1.0, val)) 
            scaled_val = val * self.power_limit_percent
            scaled_thrusters.append(int(scaled_val))
            
        return tuple(scaled_thrusters)

    # ══════════════════════════════════════════════════════════════
    #  DURUM GEÇİŞİ
    # ══════════════════════════════════════════════════════════════
    def _change_state(self, new_state):
        self.get_logger().info(f'🔄 DURUM: {self.current_state} ➜ {new_state}')
        self.current_state = new_state
        self.state_start_time = time.time()

    # ══════════════════════════════════════════════════════════════
    #  KOMUT GÖNDER (Twist → PWM → Serial + /cmd_vel yayını)
    # ══════════════════════════════════════════════════════════════
    def _send_command(self, twist):
        """Twist mesajını Base Station'a gönderir ve /cmd_vel yayınlar."""
        # 1) Twist değerlerini motor yüzdelerine çevir
        m1, m2, m3, m4, m5, m6 = self._twist_to_percentages(twist)
        
        # 2) Serial üzerinden Gönder
        self._send_serial(m1, m2, m3, m4, m5, m6)

        # 3) /cmd_vel yayınla (debug/rosbag kaydı için)
        self.cmd_pub.publish(twist)

        # 4) Konsol çıktısı
        self.get_logger().info(
            f'[{self.current_state:18s}] '
            f'M1:{m1:4d} M2:{m2:4d} M3:{m3:4d} M4:{m4:4d} M5:{m5:4d} M6:{m6:4d} | '
            f'lin.x={twist.linear.x:+.2f} lin.z={twist.linear.z:+.2f} ang.z={twist.angular.z:+.2f}'
        )

    # ══════════════════════════════════════════════════════════════
    #  ANA KONTROL DÖNGÜSÜ (STATE MACHINE)
    # ══════════════════════════════════════════════════════════════
    def _mission_loop(self):
        elapsed = time.time() - self.state_start_time
        twist = Twist()  # Varsayılan: tüm hızlar 0

        # ── DURUM 0: BEKLEME (Su yüzeyinde) ───────────────────────
        if self.current_state == self.STATE_WAIT:
            # Motorlar nötr, araç su yüzeyinde bekliyor
            if elapsed >= self.duration_wait:
                self._change_state(self.STATE_DIVE)

        # ── DURUM 1: DALIŞ (Agresif batma) ────────────────────────
        #    Araç su yüzeyinden su altına dalıyor.
        #    Sadece dikey motorlar aktif, yatay hareket yok.
        elif self.current_state == self.STATE_DIVE:
            twist.linear.z = -self.dive_speed  # Negatif = aşağı bat
            if elapsed >= self.duration_dive:
                self._change_state(self.STATE_FORWARD_1)

        # ── DURUM 2: DÜZ İLERİ GİT (1. düzlük) ──────────────────
        elif self.current_state == self.STATE_FORWARD_1:
            twist.linear.x = self.speed_forward
            twist.linear.z = -self.hold_depth_speed  # Derinlik koruma
            if elapsed >= self.duration_forward:
                self._change_state(self.STATE_TURN_1)

        # ── DURUM 3: SAĞA 90° DÖN ────────────────────────────────
        elif self.current_state == self.STATE_TURN_1:
            twist.angular.z = -self.speed_turn  # Negatif = sağa dönüş
            twist.linear.z = -self.hold_depth_speed  # Derinlik koruma
            if elapsed >= self.duration_turn_90:
                self._change_state(self.STATE_FORWARD_2)

        # ── DURUM 4: DÜZ İLERİ GİT (2. düzlük) ──────────────────
        elif self.current_state == self.STATE_FORWARD_2:
            twist.linear.x = self.speed_forward
            twist.linear.z = -self.hold_depth_speed  # Derinlik koruma
            if elapsed >= self.duration_forward:
                self._change_state(self.STATE_CIRCLE)

        # ── DURUM 5: DAİRE ÇİZ (TAM 360°) ──────────────────────
        elif self.current_state == self.STATE_CIRCLE:
            twist.linear.x  = self.circle_linear_x
            twist.angular.z = self.circle_angular_z
            twist.linear.z = -self.hold_depth_speed  # Derinlik koruma
            if elapsed >= self.duration_circle:
                self._change_state(self.STATE_TURN_AFTER_CIRCLE)

        # ── DURUM 5.5: DAİRE SONRASI 90° SAĞA DÖN ────────────────
        #    360° daire aracı aynı noktaya geri getirdi (C noktası)
        #    Şimdi yerinde 90° sağa dönüp Güneye bakıyoruz
        elif self.current_state == self.STATE_TURN_AFTER_CIRCLE:
            twist.angular.z = -self.speed_turn  # Negatif = sağa dönüş
            twist.linear.z = -self.hold_depth_speed  # Derinlik koruma
            if elapsed >= self.duration_turn_90:
                self._change_state(self.STATE_FORWARD_3)

        # ── DURUM 6: DÜZ İLERİ GİT (3. düzlük) ──────────────────
        elif self.current_state == self.STATE_FORWARD_3:
            twist.linear.x = self.speed_forward
            twist.linear.z = -self.hold_depth_speed  # Derinlik koruma
            if elapsed >= self.duration_forward:
                self._change_state(self.STATE_TURN_2)

        # ── DURUM 7: SAĞA 90° DÖN ────────────────────────────────
        elif self.current_state == self.STATE_TURN_2:
            twist.angular.z = -self.speed_turn  # Negatif = sağa dönüş
            twist.linear.z = -self.hold_depth_speed  # Derinlik koruma
            if elapsed >= self.duration_turn_90:
                self._change_state(self.STATE_FORWARD_4)

        # ── DURUM 8: DÜZ İLERİ GİT (başlangıca dönüş) ───────────
        elif self.current_state == self.STATE_FORWARD_4:
            twist.linear.x = self.speed_forward
            twist.linear.z = -self.hold_depth_speed  # Derinlik koruma
            if elapsed >= self.duration_forward:
                self._change_state(self.STATE_SURFACE)

        # ── DURUM 9: YÜZEYE ÇIKIŞ ────────────────────────────────
        #    Görev tamamlandı, araç yukarı itki ile yüzeye çıkıyor
        elif self.current_state == self.STATE_SURFACE:
            twist.linear.z = self.surface_speed  # Pozitif = yukarı çık
            if elapsed >= self.duration_surface:
                self._change_state(self.STATE_FINISHED)

        # ── GÖREV BİTTİ ──────────────────────────────────────────
        elif self.current_state == self.STATE_FINISHED:
            # Motorlar tamamen nötr, araç yüzeyde
            if elapsed < 1.0:
                self.get_logger().info('=' * 60)
                self.get_logger().info('🎯 GÖREV TAMAMLANDI! Araç yüzeye çıktı.')
                self.get_logger().info('=' * 60)

        # ── Komutu gönder ─────────────────────────────────────────
        self._send_command(twist)


# ══════════════════════════════════════════════════════════════════
#  ROS 2 ENTRY POINT
# ══════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = VideoMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Motorları durdur
        stop = Twist()
        m1, m2, m3, m4, m5, m6 = node._twist_to_percentages(stop)
        node._send_serial(m1, m2, m3, m4, m5, m6)
        node.cmd_pub.publish(stop)
        node.get_logger().info('⛔ Kullanıcı durdurdu → motorlar nötre alındı.')
    finally:
        # Serial portu kapat
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()