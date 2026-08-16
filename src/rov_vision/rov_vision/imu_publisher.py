#!/usr/bin/env python3
"""
=============================================================================
BNO085 IMU Publisher for ROS 2 (Jetson)
=============================================================================
Teknofest İnsansız Su Altı Sistemleri Yarışması — Antigravity Takımı

BNO085 9-Eksenli IMU (İvmeölçer + Jiroskop + Manyetometre)
  - Dahili Hillcrest FSP sensör füzyonu ile donanımsal Quaternion çıkışı
  - Gimbal Lock sorunu yok
  - Titreşime karşı yüksek direnç (iç algoritmalar filtreliyor)

Yayınlanan Topic:
  /imu/data (sensor_msgs/Imu) @ 50 Hz
    - orientation: Quaternion (x, y, z, w) — donanımsal sensör füzyonu
    - linear_acceleration: İvmeölçer (m/s²)
    - angular_velocity: Jiroskop (rad/s)

Bağımlılıklar:
  pip3 install adafruit-circuitpython-bno08x
=============================================================================
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time

# ── BNO085 Kütüphanesi ──────────────────────────────────────────────────────
BNO_AVAILABLE = False
try:
    import board
    import busio
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_bno08x import (
        BNO_REPORT_GAME_ROTATION_VECTOR,
        BNO_REPORT_ACCELEROMETER,
        BNO_REPORT_GYROSCOPE,
    )
    BNO_AVAILABLE = True
except ImportError:
    pass


class BNO085Publisher(Node):
    """
    BNO085 IMU sensöründen donanımsal sensör füzyonu verisi okuyarak
    ROS 2 Imu mesajı olarak yayınlar.

    BMI085'ten Farkları:
      - Quaternion doğrudan donanımdan geliyor (elle atan2 hesabı yok)
      - 9-eksen füzyon (ivme + gyro + manyetometre)
      - Çok daha düşük gürültü, low-pass filtre gereksiz
      - Roll, Pitch VE Yaw açıları kullanılabilir
    """

    def __init__(self):
        super().__init__('imu_publisher')

        # ── ROS 2 Parametreleri ──────────────────────────────────────────
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('i2c_address', 0x4A)  # BNO085 varsayılan: 0x4A (jumper ile 0x4B olabilir)

        self.i2c_bus_id = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        publish_rate = self.get_parameter('publish_rate_hz').value

        # ── ROS 2 Publisher ──────────────────────────────────────────────
        self.pub = self.create_publisher(Imu, '/imu/data', 10)

        # ── BNO085 Başlatma ──────────────────────────────────────────────
        self.bno = None

        if not BNO_AVAILABLE:
            self.get_logger().error(
                "adafruit-circuitpython-bno08x kutuphanesi bulunamadi!\n"
                "Lutfen kurun: pip3 install adafruit-circuitpython-bno08x"
            )
        else:
            try:
                # I2C başlat (İstenen i2c_bus_id ile)
                try:
                    from adafruit_extended_bus import ExtendedI2C as I2C
                    i2c = I2C(self.i2c_bus_id)
                except ImportError:
                    self.get_logger().warn("adafruit-extended-bus bulunamadi, varsayilan I2C kullaniliyor (pip3 install adafruit-extended-bus)")
                    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)

                self.bno = BNO08X_I2C(i2c, address=self.i2c_address)

                self.get_logger().info(
                    f"✅ BNO085 baslatildi! I2C Adres: 0x{self.i2c_address:02X}"
                )

                # Sensör raporlarını etkinleştir
                self.bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
                self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
                self.bno.enable_feature(BNO_REPORT_GYROSCOPE)

                self.get_logger().info(
                    "  📡 Aktif raporlar: GAME_ROTATION_VECTOR + ACCELEROMETER + GYROSCOPE\n"
                    "  🎯 Donanımsal sensör füzyonu (Hillcrest FSP) aktif\n"
                    f"  🔄 Yayın frekansı: {publish_rate} Hz"
                )

                # Sensörün stabilize olması için kısa bekleme
                time.sleep(0.1)

            except Exception as e:
                self.get_logger().error(
                    f"BNO085 baslatma hatasi: {e}\n"
                    f"  I2C adresini kontrol edin: sudo i2cdetect -y {self.i2c_bus_id}\n"
                    f"  Beklenen adres: 0x{self.i2c_address:02X}"
                )
                self.bno = None

        # ── Timer ────────────────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        # ── İstatistik sayaçları ─────────────────────────────────────────
        self._msg_count = 0
        self._last_log_time = time.monotonic()

    def timer_callback(self):
        """
        BNO085'ten veri okuyup ROS 2 Imu mesajı olarak yayınlar.
        """
        if self.bno is None:
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        try:
            # ── Quaternion (Donanımsal Sensör Füzyonu) ────────────────────
            # BNO085 quaternion: (i, j, k, real) formatında döner
            quat = self.bno.game_quaternion
            if quat is not None and quat[0] is not None:
                quat_i, quat_j, quat_k, quat_real = quat
                # ROS 2 Imu mesajı: (x, y, z, w) formatında
                msg.orientation.x = float(quat_i)
                msg.orientation.y = float(quat_j)
                msg.orientation.z = float(quat_k)
                msg.orientation.w = float(quat_real)
                # Orientation covariance: 0 = bilinmiyor, küçük değer = güvenilir
                msg.orientation_covariance[0] = 0.01
                msg.orientation_covariance[4] = 0.01
                msg.orientation_covariance[8] = 0.01
            else:
                # Quaternion henüz hazır değil — varsayılan birim quaternion
                msg.orientation.w = 1.0
                msg.orientation_covariance[0] = -1.0  # Veri yok

            # ── İvmeölçer (m/s²) ─────────────────────────────────────────
            accel = self.bno.acceleration
            if accel is not None and accel[0] is not None:
                msg.linear_acceleration.x = float(accel[0])
                msg.linear_acceleration.y = float(accel[1])
                msg.linear_acceleration.z = float(accel[2])

            # ── Jiroskop (rad/s) ─────────────────────────────────────────
            gyro = self.bno.gyro
            if gyro is not None and gyro[0] is not None:
                msg.angular_velocity.x = float(gyro[0])
                msg.angular_velocity.y = float(gyro[1])
                msg.angular_velocity.z = float(gyro[2])

            self.pub.publish(msg)
            self._msg_count += 1

            # Her 10 saniyede bir durum logu
            now = time.monotonic()
            if now - self._last_log_time >= 10.0:
                hz = self._msg_count / (now - self._last_log_time)
                # Quaternion'dan euler hesapla (sadece log için)
                r, p, y = self._quat_to_euler(
                    msg.orientation.x, msg.orientation.y,
                    msg.orientation.z, msg.orientation.w
                )
                self.get_logger().info(
                    f"BNO085 | {hz:.1f} Hz | "
                    f"Roll: {r:.1f}° Pitch: {p:.1f}° Yaw: {y:.1f}°"
                )
                self._msg_count = 0
                self._last_log_time = now

        except Exception as e:
            self.get_logger().warn(
                f"BNO085 veri okuma hatasi: {e}",
                throttle_duration_sec=2.0
            )

    @staticmethod
    def _quat_to_euler(x, y, z, w):
        """
        Quaternion (x, y, z, w) → Euler (Roll, Pitch, Yaw) derece cinsinden.
        Sadece log/debug amaçlı kullanılır.
        """
        # Roll (X ekseni etrafında dönüş)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        # Pitch (Y ekseni etrafında dönüş)
        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.degrees(math.asin(sinp))

        # Yaw (Z ekseni etrafında dönüş)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = BNO085Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
