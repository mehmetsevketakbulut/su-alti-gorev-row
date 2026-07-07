"""
Teknofest İnsansız Su Altı Sistemleri - Video Kanıtı Görevi
Dead Reckoning (Kör Sürüş) Tabanlı State Machine

── MOTOR KONTROLÜ ──
autonomous_driver.py ile BİREBİR AYNI Serial PWM protokolü kullanılır:
  - Serial port : COM8 (parametre ile değiştirilebilir)
  - Baud rate   : 115200
  - Paket formatı: "x1,y1,x2,y2\n"
  - PWM aralığı : 1060-1940 (nötr: 1500)
  - Kanal eşlemeleri:
      x1 → Dönüş   (angular.z)   → Sağ dönüş = düşük PWM
      y1 → İleri    (linear.x)    → İleri = düşük PWM (joystick tersi)
      x2 → Yanaşma  (linear.y)    → Nötr (kullanılmıyor)
      y2 → Derinlik (linear.z)    → Nötr (kullanılmıyor)

── PARKUR SIRASI ──
  DURUM 1 - WAIT       : Başlangıç alanında bekle
  DURUM 2 - FORWARD_1  : Düz ileri git
  DURUM 3 - TURN_1     : Sağa 90° dön
  DURUM 4 - FORWARD_2  : Düz ileri git
  DURUM 5 - CIRCLE     : 360° daire çiz (linear.x + angular.z)
  DURUM 6 - FORWARD_3  : Düz ileri git
  DURUM 7 - TURN_2     : Sağa 90° dön
  DURUM 8 - FORWARD_4  : Düz ileri git → başlangıca dön → motorları durdur

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
    STATE_WAIT       = 'WAIT'          # Durum 1
    STATE_FORWARD_1  = 'FORWARD_1'     # Durum 2
    STATE_TURN_1     = 'TURN_1'        # Durum 3
    STATE_FORWARD_2  = 'FORWARD_2'     # Durum 4
    STATE_CIRCLE     = 'CIRCLE'        # Durum 5
    STATE_FORWARD_3  = 'FORWARD_3'     # Durum 6
    STATE_TURN_2     = 'TURN_2'        # Durum 7
    STATE_FORWARD_4  = 'FORWARD_4'     # Durum 8
    STATE_FINISHED   = 'FINISHED'

    # ── PWM sabitleri (autonomous_driver.py ile birebir aynı) ────
    PWM_MIN     = 1060
    PWM_MAX     = 1940
    PWM_NEUTRAL = 1500

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
        self.declare_parameter('duration_forward',  15.0)
        self.declare_parameter('duration_turn_90',   2.5)
        self.declare_parameter('duration_circle',   12.0)

        # ── Hız parametreleri (Twist -1.0 ... +1.0 arası) ────────
        self.declare_parameter('speed_forward',      0.4)
        self.declare_parameter('speed_turn',         0.5)
        self.declare_parameter('circle_linear_x',    0.3)
        self.declare_parameter('circle_angular_z',   0.52)

        # ── PWM dönüşümü için max aralık (autonomous_driver ile aynı)
        self.declare_parameter('max_linear',         0.5)
        self.declare_parameter('max_angular',        0.8)

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
        self.get_logger().info(f'   PWM aralığı         : {self.PWM_MIN}-{self.PWM_MAX} (nötr: {self.PWM_NEUTRAL})')
        self.get_logger().info(f'   Bekleme süresi      : {self.duration_wait:.1f} s')
        self.get_logger().info(f'   Düz gitme süresi    : {self.duration_forward:.1f} s')
        self.get_logger().info(f'   90° dönüş süresi    : {self.duration_turn_90:.2f} s')
        self.get_logger().info(f'   Daire süresi        : {self.duration_circle:.1f} s')
        self.get_logger().info(f'   Düz gitme hızı      : {self.speed_forward:.2f}')
        self.get_logger().info(f'   Dönüş hızı          : {self.speed_turn:.2f}')
        self.get_logger().info(f'   Daire linear.x      : {self.circle_linear_x:.2f}')
        self.get_logger().info(f'   Daire angular.z     : {self.circle_angular_z:.2f}')
        self.get_logger().info('=' * 60)

    # ══════════════════════════════════════════════════════════════
    #  PARAMETRE OKUMA
    # ══════════════════════════════════════════════════════════════
    def _read_parameters(self):
        self.serial_port      = self.get_parameter('serial_port').value
        self.baud_rate        = self.get_parameter('baud_rate').value

        self.duration_wait    = self.get_parameter('duration_wait').value
        self.duration_forward = self.get_parameter('duration_forward').value
        self.duration_turn_90 = self.get_parameter('duration_turn_90').value
        self.duration_circle  = self.get_parameter('duration_circle').value

        self.speed_forward    = self.get_parameter('speed_forward').value
        self.speed_turn       = self.get_parameter('speed_turn').value
        self.circle_linear_x  = self.get_parameter('circle_linear_x').value
        self.circle_angular_z = self.get_parameter('circle_angular_z').value

        self.max_linear       = self.get_parameter('max_linear').value
        self.max_angular      = self.get_parameter('max_angular').value
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

    # ══════════════════════════════════════════════════════════════
    #  TWIST → PWM DÖNÜŞÜMÜ (autonomous_driver.py ile birebir aynı)
    # ══════════════════════════════════════════════════════════════
    def _twist_to_pwm(self, twist):
        """
        autonomous_driver.py'deki _twist_to_pwm ile AYNI mantık:
          angular.z → x1 (dönüş)  : pozitif=sola → yüksek PWM=sağa
          linear.x  → y1 (ileri)  : pozitif=ileri → düşük PWM (joystick tersi)
          linear.y  → x2 (yanaşma): nötr
          linear.z  → y2 (derinlik): nötr
        """
        # Angular.z → x1 (dönüş)
        ang_z = clamp(twist.angular.z, -self.max_angular, self.max_angular)
        x1 = map_value(ang_z, -self.max_angular, self.max_angular,
                        self.PWM_MAX, self.PWM_MIN)

        # Linear.x → y1 (ileri/geri) — joystick Y ekseni ters!
        lin_x = clamp(twist.linear.x, -self.max_linear, self.max_linear)
        y1 = map_value(lin_x, -self.max_linear, self.max_linear,
                        self.PWM_MAX, self.PWM_MIN)

        # Yanaşma ve derinlik kullanılmıyor → nötr
        x2 = self.PWM_NEUTRAL
        y2 = self.PWM_NEUTRAL

        return x1, y1, x2, y2

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
        """Twist mesajını hem Serial PWM olarak Arduino'ya hem de /cmd_vel'e gönderir."""
        # 1) Serial PWM gönder (ASIL MOTOR KONTROLÜ)
        x1, y1, x2, y2 = self._twist_to_pwm(twist)
        self._send_serial(x1, y1, x2, y2)

        # 2) /cmd_vel yayınla (debug/rosbag kaydı için)
        self.cmd_pub.publish(twist)

        # 3) Konsol çıktısı
        self.get_logger().info(
            f'[{self.current_state:12s}] '
            f'PWM: x1={x1:4d} y1={y1:4d} x2={x2:4d} y2={y2:4d} | '
            f'lin.x={twist.linear.x:+.2f} ang.z={twist.angular.z:+.2f}'
        )

    # ══════════════════════════════════════════════════════════════
    #  ANA KONTROL DÖNGÜSÜ (STATE MACHINE)
    # ══════════════════════════════════════════════════════════════
    def _mission_loop(self):
        elapsed = time.time() - self.state_start_time
        twist = Twist()  # Varsayılan: tüm hızlar 0

        # ── DURUM 1: BEKLEME ──────────────────────────────────────
        if self.current_state == self.STATE_WAIT:
            if elapsed >= self.duration_wait:
                self._change_state(self.STATE_FORWARD_1)

        # ── DURUM 2: DÜZ İLERİ GİT (1. düzlük) ──────────────────
        elif self.current_state == self.STATE_FORWARD_1:
            twist.linear.x = self.speed_forward
            if elapsed >= self.duration_forward:
                self._change_state(self.STATE_TURN_1)

        # ── DURUM 3: SAĞA 90° DÖN ────────────────────────────────
        elif self.current_state == self.STATE_TURN_1:
            twist.angular.z = -self.speed_turn  # Negatif = sağa dönüş
            if elapsed >= self.duration_turn_90:
                self._change_state(self.STATE_FORWARD_2)

        # ── DURUM 4: DÜZ İLERİ GİT (2. düzlük) ──────────────────
        elif self.current_state == self.STATE_FORWARD_2:
            twist.linear.x = self.speed_forward
            if elapsed >= self.duration_forward:
                self._change_state(self.STATE_CIRCLE)

        # ── DURUM 5: DAİRE ÇİZ (360°) ────────────────────────────
        elif self.current_state == self.STATE_CIRCLE:
            twist.linear.x  = self.circle_linear_x
            twist.angular.z = self.circle_angular_z
            if elapsed >= self.duration_circle:
                self._change_state(self.STATE_FORWARD_3)

        # ── DURUM 6: DÜZ İLERİ GİT (3. düzlük) ──────────────────
        elif self.current_state == self.STATE_FORWARD_3:
            twist.linear.x = self.speed_forward
            if elapsed >= self.duration_forward:
                self._change_state(self.STATE_TURN_2)

        # ── DURUM 7: SAĞA 90° DÖN ────────────────────────────────
        elif self.current_state == self.STATE_TURN_2:
            twist.angular.z = -self.speed_turn  # Negatif = sağa dönüş
            if elapsed >= self.duration_turn_90:
                self._change_state(self.STATE_FORWARD_4)

        # ── DURUM 8: DÜZ İLERİ GİT (başlangıca dönüş) ───────────
        elif self.current_state == self.STATE_FORWARD_4:
            twist.linear.x = self.speed_forward
            if elapsed >= self.duration_forward:
                self._change_state(self.STATE_FINISHED)

        # ── GÖREV BİTTİ ──────────────────────────────────────────
        elif self.current_state == self.STATE_FINISHED:
            # Motorlar zaten 0 (twist varsayılanı)
            if elapsed < 1.0:
                self.get_logger().info('=' * 60)
                self.get_logger().info('🎯 GÖREV TAMAMLANDI! Araç başlangıç alanına döndü.')
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
        x1, y1, x2, y2 = node._twist_to_pwm(stop)
        node._send_serial(x1, y1, x2, y2)
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