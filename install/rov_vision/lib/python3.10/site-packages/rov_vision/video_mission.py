"""
Teknofest İnsansız Su Altı Sistemleri - Video Kanıtı Görevi
Dead Reckoning (Kör Sürüş) Tabanlı State Machine

Parkur Sırası:
  DURUM 1 - WAIT        : Başlangıç alanında 5 sn bekle
  DURUM 2 - FORWARD_1   : 15 sn düz ileri git
  DURUM 3 - TURN_1      : Sağa 90° dön
  DURUM 4 - FORWARD_2   : 15 sn düz ileri git
  DURUM 5 - CIRCLE       : 360° daire çiz (linear.x + angular.z)
  DURUM 6 - FORWARD_3   : 15 sn düz ileri git
  DURUM 7 - TURN_2      : Sağa 90° dön
  DURUM 8 - FORWARD_4   : 15 sn düz ileri git → başlangıca dön → motorları durdur

Kamera / Sensör (IMU) abonesi YOKTUR.
Sadece '/cmd_vel' üzerinden Twist mesajı yayınlanır.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


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

    def __init__(self):
        super().__init__('video_mission_node')

        # ==========================================================
        #  ROS 2 PARAMETRELERİ  (Havuz kenarında yaml veya CLI ile değiştir)
        # ==========================================================

        # ── Süre parametreleri (saniye) ───────────────────────────
        self.declare_parameter('duration_wait',      5.0)   # Başlangıç bekleme
        self.declare_parameter('duration_forward',  15.0)   # Düz gitme süresi
        self.declare_parameter('duration_turn_90',   2.5)   # 90° dönüş süresi (havuzda kalibre et!)
        self.declare_parameter('duration_circle',   12.0)   # 360° daire tamamlama süresi

        # ── Hız parametreleri ─────────────────────────────────────
        self.declare_parameter('speed_forward',      0.4)   # Düz gitme linear.x hızı
        self.declare_parameter('speed_turn',         0.5)   # 90° dönüş angular.z hızı (+ = sola, - = sağa)
        self.declare_parameter('circle_linear_x',    0.3)   # Daire çizerken ileri hız
        self.declare_parameter('circle_angular_z',   0.52)  # Daire çizerken dönüş hızı (2π / duration_circle ≈ 0.52)

        # ── Timer frekansı ────────────────────────────────────────
        self.declare_parameter('loop_rate_hz',      20.0)   # Kontrol döngü frekansı

        # ── Parametreleri iç değişkenlere oku ─────────────────────
        self._read_parameters()

        # ── Durum makinesi değişkenleri ───────────────────────────
        self.current_state = self.STATE_WAIT
        self.state_start_time = time.time()

        # ── Publisher ─────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Kontrol döngüsü ──────────────────────────────────────
        loop_period = 1.0 / self.loop_rate_hz
        self.timer = self.create_timer(loop_period, self._mission_loop)

        self.get_logger().info('='*60)
        self.get_logger().info('🚀 Teknofest Video Görev Kontrolcüsü HAZIR!')
        self.get_logger().info(f'   Bekleme süresi      : {self.duration_wait:.1f} s')
        self.get_logger().info(f'   Düz gitme süresi    : {self.duration_forward:.1f} s')
        self.get_logger().info(f'   90° dönüş süresi    : {self.duration_turn_90:.2f} s')
        self.get_logger().info(f'   Daire süresi        : {self.duration_circle:.1f} s')
        self.get_logger().info(f'   Düz gitme hızı      : {self.speed_forward:.2f}')
        self.get_logger().info(f'   Dönüş hızı          : {self.speed_turn:.2f}')
        self.get_logger().info(f'   Daire linear.x      : {self.circle_linear_x:.2f}')
        self.get_logger().info(f'   Daire angular.z     : {self.circle_angular_z:.2f}')
        self.get_logger().info('='*60)

    # ──────────────────────────────────────────────────────────────
    #  Parametre Okuma
    # ──────────────────────────────────────────────────────────────
    def _read_parameters(self):
        """ROS 2 parametrelerini iç değişkenlere atar."""
        self.duration_wait     = self.get_parameter('duration_wait').value
        self.duration_forward  = self.get_parameter('duration_forward').value
        self.duration_turn_90  = self.get_parameter('duration_turn_90').value
        self.duration_circle   = self.get_parameter('duration_circle').value

        self.speed_forward     = self.get_parameter('speed_forward').value
        self.speed_turn        = self.get_parameter('speed_turn').value
        self.circle_linear_x   = self.get_parameter('circle_linear_x').value
        self.circle_angular_z  = self.get_parameter('circle_angular_z').value

        self.loop_rate_hz      = self.get_parameter('loop_rate_hz').value

    # ──────────────────────────────────────────────────────────────
    #  Durum Geçişi
    # ──────────────────────────────────────────────────────────────
    def _change_state(self, new_state: str):
        """Mevcut durumu değiştirir ve zamanlayıcıyı sıfırlar."""
        self.get_logger().info(f'🔄 DURUM GEÇİŞİ: {self.current_state} ➜ {new_state}')
        self.current_state = new_state
        self.state_start_time = time.time()

    # ──────────────────────────────────────────────────────────────
    #  Yardımcı: Twist oluştur
    # ──────────────────────────────────────────────────────────────
    @staticmethod
    def _make_twist(linear_x: float = 0.0, angular_z: float = 0.0) -> Twist:
        """Verilen hız değerleriyle bir Twist mesajı döndürür."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        return twist

    # ──────────────────────────────────────────────────────────────
    #  Ana Kontrol Döngüsü  (State Machine)
    # ──────────────────────────────────────────────────────────────
    def _mission_loop(self):
        elapsed = time.time() - self.state_start_time
        twist = Twist()   # Varsayılan: tüm hızlar 0

        # ── DURUM 1: BEKLEME ──────────────────────────────────────
        if self.current_state == self.STATE_WAIT:
            # Başlangıç alanında sabit dur
            if elapsed >= self.duration_wait:
                self._change_state(self.STATE_FORWARD_1)

        # ── DURUM 2: DÜZ İLERİ GİT (1. düzlük) ──────────────────
        elif self.current_state == self.STATE_FORWARD_1:
            twist = self._make_twist(linear_x=self.speed_forward)
            if elapsed >= self.duration_forward:
                self._change_state(self.STATE_TURN_1)

        # ── DURUM 3: SAĞA 90° DÖN (1. dönüş) ────────────────────
        elif self.current_state == self.STATE_TURN_1:
            # Sağa dönmek için angular.z negatif (REP-103 kuralı)
            twist = self._make_twist(angular_z=-self.speed_turn)
            if elapsed >= self.duration_turn_90:
                self._change_state(self.STATE_FORWARD_2)

        # ── DURUM 4: DÜZ İLERİ GİT (2. düzlük) ──────────────────
        elif self.current_state == self.STATE_FORWARD_2:
            twist = self._make_twist(linear_x=self.speed_forward)
            if elapsed >= self.duration_forward:
                self._change_state(self.STATE_CIRCLE)

        # ── DURUM 5: DAİRE ÇİZ (360°) ────────────────────────────
        # Hem ileri hız hem dönüş hızı aynı anda verilerek
        # minimum 1 metre çaplı bir daire çizilir.
        # Daire çapı ≈ 2 × linear_x / angular_z formülü ile ayarlanır.
        elif self.current_state == self.STATE_CIRCLE:
            twist = self._make_twist(
                linear_x=self.circle_linear_x,
                angular_z=self.circle_angular_z
            )
            if elapsed >= self.duration_circle:
                self._change_state(self.STATE_FORWARD_3)

        # ── DURUM 6: DÜZ İLERİ GİT (3. düzlük) ──────────────────
        elif self.current_state == self.STATE_FORWARD_3:
            twist = self._make_twist(linear_x=self.speed_forward)
            if elapsed >= self.duration_forward:
                self._change_state(self.STATE_TURN_2)

        # ── DURUM 7: SAĞA 90° DÖN (2. dönüş) ────────────────────
        elif self.current_state == self.STATE_TURN_2:
            twist = self._make_twist(angular_z=-self.speed_turn)
            if elapsed >= self.duration_turn_90:
                self._change_state(self.STATE_FORWARD_4)

        # ── DURUM 8: DÜZ İLERİ GİT (başlangıca dönüş) ───────────
        elif self.current_state == self.STATE_FORWARD_4:
            twist = self._make_twist(linear_x=self.speed_forward)
            if elapsed >= self.duration_forward:
                self._change_state(self.STATE_FINISHED)

        # ── GÖREV BİTTİ ──────────────────────────────────────────
        elif self.current_state == self.STATE_FINISHED:
            # Motorları tamamen durdur
            twist = self._make_twist()
            if elapsed < 1.0:
                self.get_logger().info('='*60)
                self.get_logger().info('🎯 GÖREV TAMAMLANDI! Araç başlangıç alanına döndü.')
                self.get_logger().info('   Motorlar kapatıldı.')
                self.get_logger().info('='*60)

        # ── Komutu yayınla ────────────────────────────────────────
        self.cmd_pub.publish(twist)


# ══════════════════════════════════════════════════════════════════
#  ROS 2 Entry Point
# ══════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = VideoMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C ile çıkışta motorları durdur
        stop_twist = Twist()
        node.cmd_pub.publish(stop_twist)
        node.get_logger().info('⛔ Kullanıcı durdurdu → motorlar kapatıldı.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()