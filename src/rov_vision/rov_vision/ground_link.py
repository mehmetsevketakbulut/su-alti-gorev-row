#!/usr/bin/env python3
"""
=============================================================================
ALTAY ROV - GROUND LINK  (Jetson Orin Nano, ROS 2)
=============================================================================
Yer istasyonu (PC) ile ROS 2 arasindaki UDP koprusu.

  PC  --UDP 5005 (JSON)-->  bu node  -->  /rov/manual, /rov/mode,
                                          /rov/armed, /rov/aux
  PC  <--UDP 5006 (JSON)--  bu node  <--  /rov/telemetry, /depth_sensor,
                                          /distance_sensor, /imu/data

Seri porta DOKUNMAZ. Motor karisimi YAPMAZ. Sadece cevirir.

CALISTIRMA:
  ros2 run altay_rov ground_link
  ros2 run altay_rov ground_link --ros-args -p pc_ip:=192.168.1.5
=============================================================================
"""

import json
import math
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32, Int32MultiArray


class GroundLink(Node):

    def __init__(self):
        super().__init__('ground_link')

        self.declare_parameter('pc_ip', '192.168.1.50')
        self.declare_parameter('listen_port', 5005)
        self.declare_parameter('telemetry_port', 5006)
        self.declare_parameter('telemetry_rate_hz', 10.0)

        self.pc_ip = self.get_parameter('pc_ip').value
        lport = self.get_parameter('listen_port').value
        self.tport = self.get_parameter('telemetry_port').value
        trate = self.get_parameter('telemetry_rate_hz').value

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        self.pub_manual = self.create_publisher(Twist, '/rov/manual', qos)
        self.pub_mode = self.create_publisher(Int32, '/rov/mode', 10)
        self.pub_armed = self.create_publisher(Bool, '/rov/armed', 10)
        self.pub_aux = self.create_publisher(Int32MultiArray, '/rov/aux', 10)

        self.telem = [0.0] * 11
        self.depth_cm = 0.0
        self.dist_cm = 0.0
        self.roll = self.pitch = 0.0

        self.create_subscription(Float32MultiArray, '/rov/telemetry',
                                 self._cb_telem, 10)
        self.create_subscription(Float32, '/depth_sensor', self._cb_depth, qos)
        self.create_subscription(Float32, '/distance_sensor', self._cb_dist, qos)
        self.create_subscription(Imu, '/imu/data', self._cb_imu, qos)

        self.sock_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_in.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
        self.sock_in.bind(('0.0.0.0', lport))
        self.sock_in.setblocking(False)
        self.sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._son_seq = -1
        self.create_timer(0.005, self._udp_oku)          # 200 Hz drenaj
        self.create_timer(1.0 / trate, self._telemetri_gonder)

        self.get_logger().info(
            f'Ground Link hazir. Dinleniyor :{lport}, PC {self.pc_ip}:{self.tport}'
        )

    # ------------------------------------------------------------ ROS -> ic
    def _cb_telem(self, msg):
        self.telem = list(msg.data)

    def _cb_depth(self, msg):
        self.depth_cm = float(msg.data)

    def _cb_dist(self, msg):
        self.dist_cm = float(msg.data)

    def _cb_imu(self, msg):
        q = msg.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.roll = math.degrees(math.atan2(sinr, cosr))
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        self.pitch = math.degrees(math.asin(sinp))

    # ------------------------------------------------------------ UDP -> ROS
    def _udp_oku(self):
        yeni = None
        # Kuyrugu TAMAMEN bosalt; sadece en yeni paket gecerli olsun,
        # yoksa yuk altinda gecikme birikir.
        while True:
            try:
                data, _ = self.sock_in.recvfrom(4096)
            except (BlockingIOError, InterruptedError, OSError):
                break
            try:
                p = json.loads(data.decode('utf-8'))
            except Exception:
                continue
            s = p.get('seq', 0)
            # Sirasiz gelen ESKI paketi at (UDP sira garantisi vermez)
            if self._son_seq >= 0 and 0 < (self._son_seq - s) < 1_000_000:
                continue
            self._son_seq = s
            yeni = p

        if yeni is None:
            return

        t = Twist()
        t.linear.x = float(yeni.get('fwd', 0)) / 100.0
        t.linear.y = float(yeni.get('strafe', 0)) / 100.0
        t.linear.z = float(yeni.get('dive', 0)) / 100.0
        t.angular.z = float(yeni.get('yaw', 0)) / 100.0
        self.pub_manual.publish(t)

        m = Int32(); m.data = int(yeni.get('mod', 0)); self.pub_mode.publish(m)
        a = Bool(); a.data = bool(yeni.get('armed', 0)); self.pub_armed.publish(a)

        aux = Int32MultiArray()
        aux.data = [
            int(yeni.get('kamera_role', 0)),
            int(yeni.get('kamera_tilt', 45)),
            int(yeni.get('magnet_evt', 0)),
            int(yeni.get('isik_ana', 0)),
            int(yeni.get('isik_mini', 0)),
        ]
        self.pub_aux.publish(aux)

    # ------------------------------------------------------------ ROS -> UDP
    def _telemetri_gonder(self):
        t = self.telem if len(self.telem) >= 11 else [0.0] * 11
        paket = {
            'servo': t[0],
            'motorlar': [int(x) for x in t[1:7]],
            'failsafe': int(t[7]),
            'magnet': int(t[8]),
            'can_err': int(t[9]),
            'uart_ok': int(t[10]),
            'derinlik_cm': round(self.depth_cm, 1),
            'mesafe_cm': round(self.dist_cm, 1),
            'roll': round(self.roll, 1),
            'pitch': round(self.pitch, 1),
            't': time.time(),
        }
        try:
            self.sock_out.sendto(json.dumps(paket).encode(),
                                 (self.pc_ip, self.tport))
        except Exception:
            pass

    def destroy_node(self):
        try:
            self.sock_in.close()
            self.sock_out.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GroundLink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
