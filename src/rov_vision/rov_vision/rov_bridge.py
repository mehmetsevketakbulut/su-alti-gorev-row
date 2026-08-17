#!/usr/bin/env python3
"""
=============================================================================
ALTAY ROV - ROV BRIDGE  (Jetson Orin Nano, ROS 2)
=============================================================================
SERI PORTU ACAN TEK NODE BUDUR. Baska hicbir node /dev/ttyUSB* acmamalidir.
(Eski kodlarda autonomous_driver.py ve mission_navigation.py ayni portu
acmaya calisiyordu; ikisi birlikte calisinca ikincisi "device busy" aliyordu.)

MUX de bu node'un icinde - ayri dosya yok, daha az hareketli parca:
   mod 0 -> /rov/manual   (joystick)
   mod 1 -> /cmd_vel      (line_follower otonom)
   mod 2 -> /rov/manual   (joystick, hedef = Mini ROV)
   LB'ye basinca mod 0'a doner -> otonom aninda devre disi (override)

ABONE:
  /rov/manual        geometry_msgs/Twist        joystick eksenleri (-1..1)
  /rov/mode          std_msgs/Int32             0 / 1 / 2
  /rov/armed         std_msgs/Bool
  /rov/aux           std_msgs/Int32MultiArray   [role, tilt, magnet_evt,
                                                 isik_ana, isik_mini]
  /cmd_vel           geometry_msgs/Twist        otonom surus
  /line_follower/tilt std_msgs/Int32            otonom kamera tarama acisi
  /imu/data          sensor_msgs/Imu            roll stabilizasyonu icin

YAYIN:
  /rov/telemetry     std_msgs/Float32MultiArray
     [servo, m1..m6, failsafe, magnet, can_err, uart_ok]

CALISTIRMA:
  ros2 run altay_rov rov_bridge
  ros2 run altay_rov rov_bridge --ros-args -p serial_port:=/dev/ttyUSB0
=============================================================================
"""

import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import serial
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32MultiArray, Int32, Int32MultiArray

SYNC1, SYNC2, PAYLOAD_LEN, FRAME_LEN = 0xAA, 0x55, 12, 17

# CRC8 poly 0x07, init 0x00 - firmware ile birebir ayni
_CRC = []
for _b in range(256):
    _c = _b
    for _ in range(8):
        _c = ((_c << 1) ^ 0x07) & 0xFF if (_c & 0x80) else ((_c << 1) & 0xFF)
    _CRC.append(_c)


def crc8(data: bytes) -> int:
    c = 0
    for b in data:
        c = _CRC[c ^ b]
    return c


def kirp(v, lo=-100, hi=100):
    return int(max(lo, min(hi, v)))


class RovBridge(Node):

    def __init__(self):
        super().__init__('rov_bridge')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('send_rate_hz', 50.0)
        self.declare_parameter('manual_timeout_s', 0.4)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        rate = self.get_parameter('send_rate_hz').value
        self.manual_timeout = self.get_parameter('manual_timeout_s').value

        # ---- Durum ----
        self.mode = 0
        self.armed = False
        self.manual = Twist()
        self.auto = Twist()
        self.kamera_role = 0
        self.tilt_manual = 45
        self.tilt_auto = 45
        self.magnet_evt = 0
        self.isik_ana = 0
        self.isik_mini = 0
        self.roll_deg = 0.0
        self.seq = 0

        self._son_manual = 0.0
        self._son_auto = 0.0
        self._lock = threading.Lock()

        self.telem = [0.0] * 11
        self._rx = bytearray()

        # ---- Seri port ----
        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0)
            # ESP32 USB kopruleri DTR/RTS ile karti resetler. Bunlari
            # indirmezsek bridge her aciliste Deneyap yeniden baslar.
            self.ser.setDTR(False)
            self.ser.setRTS(False)
            self.get_logger().info(f'Seri port acildi: {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(
                f'SERI PORT ACILAMADI: {e}\n'
                f'  ls -l /dev/ttyUSB* ile portu kontrol edin.\n'
                f'  Yetki hatasi ise: sudo usermod -aG dialout $USER (sonra logout)'
            )

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(Twist, '/rov/manual', self._cb_manual, qos)
        self.create_subscription(Twist, '/cmd_vel', self._cb_auto, qos)
        self.create_subscription(Int32, '/rov/mode', self._cb_mode, 10)
        self.create_subscription(Bool, '/rov/armed', self._cb_armed, 10)
        self.create_subscription(Int32MultiArray, '/rov/aux', self._cb_aux, 10)
        self.create_subscription(Int32, '/line_follower/tilt', self._cb_tilt_auto, 10)
        self.create_subscription(Imu, '/imu/data', self._cb_imu, qos)

        self.pub_telem = self.create_publisher(Float32MultiArray, '/rov/telemetry', 10)

        self.create_timer(1.0 / rate, self._gonder)
        self.create_timer(0.02, self._telemetri_oku)

        self.get_logger().info(
            f'ROV Bridge hazir. {rate:.0f} Hz KESINTISIZ cerceve gonderilecek.\n'
            f'  UDP/komut kesilse bile cerceve akmaya devam eder; eksenler\n'
            f'  sifirlanir ve ARMED duser. Boylece Deneyap "hat sagam ama\n'
            f'  komut notr" ile "hat koptu" durumlarini ayirt edebilir.'
        )

    # ------------------------------------------------------------ callbacks
    def _cb_manual(self, msg):
        with self._lock:
            self.manual = msg
            self._son_manual = time.monotonic()

    def _cb_auto(self, msg):
        with self._lock:
            self.auto = msg
            self._son_auto = time.monotonic()

    def _cb_mode(self, msg):
        with self._lock:
            self.mode = int(msg.data)

    def _cb_armed(self, msg):
        with self._lock:
            self.armed = bool(msg.data)

    def _cb_tilt_auto(self, msg):
        with self._lock:
            self.tilt_auto = kirp(msg.data, 0, 90)

    def _cb_aux(self, msg):
        d = list(msg.data)
        with self._lock:
            if len(d) > 0: self.kamera_role = 1 if d[0] else 0
            if len(d) > 1: self.tilt_manual = kirp(d[1], 0, 90)
            if len(d) > 2: self.magnet_evt = int(d[2]) & 0xFF
            if len(d) > 3: self.isik_ana = kirp(d[3], 0, 100)
            if len(d) > 4: self.isik_mini = kirp(d[4], 0, 100)

    def _cb_imu(self, msg):
        q = msg.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        with self._lock:
            self.roll_deg = math.degrees(math.atan2(sinr, cosr))

    # --------------------------------------------------------------- MUX
    def _sec(self):
        """Hangi kaynak direksiyonda? Tek karar noktasi burasi."""
        simdi = time.monotonic()
        with self._lock:
            mod = self.mode
            armed = self.armed
            manual_taze = (simdi - self._son_manual) < self.manual_timeout
            auto_taze = (simdi - self._son_auto) < self.manual_timeout

            if not manual_taze:
                # Yer istasyonu sustu -> her sey notre, ARMED duser.
                return 1, 0, 0, 0, 0, False, self.tilt_manual

            if mod == 1:
                # Otonom. line_follower cokerse auto_taze False olur ve
                # arac kontrolsuz kalmaz - sifir basariz.
                t = self.auto if auto_taze else Twist()
                tilt = self.tilt_auto
                return 1, t.linear.x, t.linear.y, t.linear.z, t.angular.z, armed, tilt

            t = self.manual
            target = 2 if mod == 2 else 1
            return target, t.linear.x, t.linear.y, t.linear.z, t.angular.z, \
                armed, self.tilt_manual

    # ---------------------------------------------------------- gonderim
    def _gonder(self):
        if self.ser is None:
            return

        target, fx, fy, fz, fyaw, armed, tilt = self._sec()

        vx = kirp(int(fx * 100.0), -100, 100)
        vy = kirp(int(fy * 100.0), -100, 100)
        vz = kirp(int(fz * 100.0), -100, 100)
        yaw = kirp(int(fyaw * 100.0), -100, 100)

        with self._lock:
            mod = self.mode
            role = self.kamera_role
            mag = self.magnet_evt
            l_a = self.isik_ana if armed else 0
            l_m = self.isik_mini if armed else 0
            roll = kirp(int(self.roll_deg), -100, 100)
        # Motor Mixing (Vectored 6-Thruster)
        if target == 2:
            # MINI ROV MODU
            m1 = m2 = m3 = m4 = m5 = m6 = 0 # Ana ROV dursun
            fs_mini = 0 if armed else 1
            mesaj_mini = f"M,{vx},{vy},{vz},{yaw},{fs_mini},{l_m}\n"
            try:
                self.ser.write(mesaj_mini.encode('utf-8'))
                self.ser.flush()
                time.sleep(0.005) # Arduino buffer (64 byte) tasmasini engelle
            except Exception:
                pass
        else:
            # ANA ROV MODU
            m1 = int((fx - fy - fyaw) * 100)
            m2 = int((fx + fy + fyaw) * 100)
            m3 = int((fx + fy - fyaw) * 100)
            m4 = int((fx - fy + fyaw) * 100)
            m5 = int(fz * 100)
            m6 = int(fz * 100)

        # Sinirla
        m1 = kirp(m1, -100, 100)
        m2 = kirp(m2, -100, 100)
        m3 = kirp(m3, -100, 100)
        m4 = kirp(m4, -100, 100)
        m5 = kirp(m5, -100, 100)
        m6 = kirp(m6, -100, 100)
        
        if not armed:
            m1 = m2 = m3 = m4 = m5 = m6 = 0

        failsafe_flag = 0 if armed else 1
        
        # A,m1,m2,m3,m4,m5,m6,failsafe,role,miknatis,isik,roll,tilt
        mesaj = f"A,{m1},{m2},{m3},{m4},{m5},{m6},{failsafe_flag},{role},{mag},{l_a},{roll},{tilt}\n"

        try:
            self.ser.write(mesaj.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f'Seri yazma hatasi: {e}', throttle_duration_sec=2.0)

    # --------------------------------------------------------- telemetri
    def _telemetri_oku(self):
        if self.ser is None:
            return
        try:
            n = self.ser.in_waiting
            if n:
                self._rx.extend(self.ser.read(n))
        except Exception:
            return

        while b'\n' in self._rx:
            satir, _, kalan = self._rx.partition(b'\n')
            self._rx = bytearray(kalan)
            s = satir.decode('utf-8', 'ignore').strip()
            
            # AnaROV_Arac.ino'dan gelen debug mesajlarini ROS loguna yazdir
            if s.startswith('[CAN ALINDI]') or s.startswith(' -> '):
                self.get_logger().info(f"[MINI ROV DEBUG] {s}")
                continue
                
            if not s.startswith('T,'):
                continue
            p = s.split(',')
            if len(p) < 11:
                continue
            try:
                vals = [float(x) for x in p[1:11]]
            except ValueError:
                continue
            vals.append(1.0)                       # uart_ok
            self.telem = vals
            m = Float32MultiArray()
            m.data = vals
            self.pub_telem.publish(m)

        if len(self._rx) > 4096:
            self._rx = bytearray()

    def destroy_node(self):
        # Kapanirken motorlari kesin kilitle
        if self.ser:
            try:
                with self._lock:
                    self.armed = False
                for _ in range(10):
                    self._gonder()
                    time.sleep(0.02)
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RovBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
