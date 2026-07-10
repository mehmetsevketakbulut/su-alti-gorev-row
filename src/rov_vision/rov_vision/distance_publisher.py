#!/usr/bin/env python3
"""
=============================================================================
DISTANCE PUBLISHER - Su Altı Akustik Mesafe Sensörü (UART → ROS2)
=============================================================================
Teknofest İnsansız Su Altı Sistemleri Yarışması — Antigravity Takımı

Su altı akustik mesafe sensöründen (8-300 cm, UART arayüzü) gelen
mesafe verilerini ROS2 topic'ine yayınlayan node.

Desteklenen Sensör Protokolleri:
  1. BINARY (varsayılan): [0xFF] [DATA_H] [DATA_L] [CHECKSUM]
     - Mesafe mm cinsinden gelir → cm'ye çevrilir
     - Checksum: (0xFF + DATA_H + DATA_L) & 0xFF
  
  2. ASCII: Satır sonu ile biten ASCII string (ör: "125\r\n")
     - Mesafe doğrudan cm cinsinden okunur

Sensör Özellikleri:
  - Menzil: 8 cm — 300 cm (3-6 metre modele göre)
  - Arayüz: UART (TTL 3.3V / 5V)
  - Baud Rate: 9600 (çoğu model için standart)

ROS2 Topic:
  PUB: /distance_sensor (std_msgs/Float32)  — mesafe cm cinsinden

Kullanım:
  ros2 run rov_vision distance_publisher
  ros2 run rov_vision distance_publisher --ros-args -p serial_port:=/dev/ttyUSB0 -p baud_rate:=9600

=============================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import serial
import time
import threading


class DistancePublisher(Node):
    """
    UART akustik mesafe sensörü okuyucu.
    Aracın altına monte edilmiş, tahtaya/zemine olan mesafeyi ölçer.
    """

    def __init__(self):
        super().__init__('distance_publisher')

        # ── ROS2 Parametreleri ──────────────────────────────────────────
        self.declare_parameter('serial_port', 'COM9')        # Sensör UART portu
        self.declare_parameter('baud_rate', 9600)             # Çoğu akustik sensör 9600
        self.declare_parameter('publish_topic', '/distance_sensor')
        self.declare_parameter('protocol', 'binary')          # 'binary' veya 'ascii'
        self.declare_parameter('publish_rate_hz', 10.0)       # Yayın frekansı
        self.declare_parameter('min_range_cm', 8.0)           # Sensör min menzil
        self.declare_parameter('max_range_cm', 300.0)         # Sensör max menzil
        self.declare_parameter('median_filter_size', 5)       # Gürültü filtresi

        self._port = self.get_parameter('serial_port').value
        self._baud = self.get_parameter('baud_rate').value
        self._protocol = self.get_parameter('protocol').value
        self._min_range = self.get_parameter('min_range_cm').value
        self._max_range = self.get_parameter('max_range_cm').value
        self._filter_size = self.get_parameter('median_filter_size').value

        # ── Publisher ───────────────────────────────────────────────────
        topic = self.get_parameter('publish_topic').value
        self.pub = self.create_publisher(Float32, topic, 10)

        # ── Mesafe tamponu (median filtre için) ─────────────────────────
        self._distance_buffer = []

        # ── Serial port aç ──────────────────────────────────────────────
        self.ser = None
        self._open_serial()

        # ── Okuma thread'i ──────────────────────────────────────────────
        self._latest_distance_cm = -1.0
        self._running = True
        self._read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self._read_thread.start()

        # ── Yayın timer'ı ───────────────────────────────────────────────
        rate = self.get_parameter('publish_rate_hz').value
        self.create_timer(1.0 / rate, self._publish_callback)

        self.get_logger().info(
            f'📏 Distance Publisher başlatıldı!\n'
            f'  Port     : {self._port} @ {self._baud} baud\n'
            f'  Protokol : {self._protocol}\n'
            f'  Menzil   : {self._min_range}-{self._max_range} cm\n'
            f'  Topic    : {topic}\n'
            f'  Filtre   : Median ({self._filter_size} örnek)'
        )

    def _open_serial(self):
        """Serial portu açar."""
        try:
            self.ser = serial.Serial(
                self._port, self._baud,
                timeout=1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.get_logger().info(f'✅ Mesafe sensörü bağlandı: {self._port}')
        except Exception as e:
            self.get_logger().error(
                f'❌ Mesafe sensörü bağlanamadı: {e}\n'
                f'  Port: {self._port}, Baud: {self._baud}\n'
                f'  → Mesafe verisi yayınlanmayacak!'
            )
            self.ser = None

    def _serial_read_loop(self):
        """
        Arka plan thread'inde serial port okur.
        Protokole göre binary veya ASCII frame parse eder.
        """
        while self._running:
            if self.ser is None or not self.ser.is_open:
                time.sleep(1.0)
                continue

            try:
                if self._protocol == 'binary':
                    self._read_binary_frame()
                else:
                    self._read_ascii_frame()
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial okuma hatası: {e}')
                time.sleep(0.5)
            except Exception as e:
                self.get_logger().warn(f'Okuma hatası: {e}')
                time.sleep(0.1)

    def _read_binary_frame(self):
        """
        Binary protokol frame okur.
        Frame: [0xFF] [DATA_H] [DATA_L] [CHECKSUM]
        Mesafe mm cinsinden → cm'ye çevrilir.
        """
        # Header byte'ı ara (0xFF)
        byte = self.ser.read(1)
        if len(byte) == 0:
            return

        if byte[0] != 0xFF:
            return  # Header değil, atla

        # Geri kalan 3 byte'ı oku
        data = self.ser.read(3)
        if len(data) < 3:
            return  # Eksik frame

        data_h = data[0]
        data_l = data[1]
        checksum = data[2]

        # Checksum doğrula
        expected_checksum = (0xFF + data_h + data_l) & 0xFF
        if checksum != expected_checksum:
            self.get_logger().debug(
                f'Checksum hatası: beklenen={expected_checksum:#04x}, '
                f'gelen={checksum:#04x}'
            )
            return

        # Mesafe hesapla (mm → cm)
        distance_mm = (data_h << 8) | data_l
        distance_cm = distance_mm / 10.0

        self._process_distance(distance_cm)

    def _read_ascii_frame(self):
        """
        ASCII protokol: satır sonu ile biten mesafe değeri (cm).
        Örnek: "125\r\n" → 125 cm
        """
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            return

        try:
            distance_cm = float(line)
            self._process_distance(distance_cm)
        except ValueError:
            self.get_logger().debug(f'ASCII parse hatası: "{line}"')

    def _process_distance(self, distance_cm):
        """
        Okunan mesafeyi filtreler ve saklar.
        Menzil dışı değerleri atar.
        """
        # Menzil kontrolü
        if distance_cm < self._min_range or distance_cm > self._max_range:
            return

        # Median filtre tamponuna ekle
        self._distance_buffer.append(distance_cm)
        if len(self._distance_buffer) > self._filter_size:
            self._distance_buffer.pop(0)

        # Median hesapla
        if len(self._distance_buffer) >= 3:
            sorted_buf = sorted(self._distance_buffer)
            median = sorted_buf[len(sorted_buf) // 2]
            self._latest_distance_cm = median
        else:
            self._latest_distance_cm = distance_cm

    def _publish_callback(self):
        """Timer callback: son mesafe değerini ROS2 topic'ine yayınlar."""
        if self._latest_distance_cm < 0:
            return  # Henüz geçerli veri yok

        msg = Float32()
        msg.data = self._latest_distance_cm
        self.pub.publish(msg)

    def destroy_node(self):
        """Temiz kapanış."""
        self._running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Mesafe sensörü serial port kapatıldı.')
        super().destroy_node()


# =============================================================================
# MAIN
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = DistancePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Distance Publisher durduruldu.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
