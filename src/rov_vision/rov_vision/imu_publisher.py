#!/usr/bin/env python3
"""
BMI085 IMU Publisher for ROS 2 (Jetson)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time

try:
    import smbus2
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False

# BMI085 I2C Adresleri (Varsayılan)
BMI085_ACCEL_ADDR = 0x18
BMI085_GYRO_ADDR = 0x68

class BMI085Publisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('accel_addr', BMI085_ACCEL_ADDR)
        self.declare_parameter('gyro_addr', BMI085_GYRO_ADDR)

        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.accel_addr = self.get_parameter('accel_addr').value
        self.gyro_addr = self.get_parameter('gyro_addr').value
        publish_rate = self.get_parameter('publish_rate_hz').value

        self.pub = self.create_publisher(Imu, '/imu/data', 10)

        if not SMBUS_AVAILABLE:
            self.get_logger().error("smbus2 kutuphanesi bulunamadi! Lutfen kurun: pip3 install smbus2")
            self.bus = None
        else:
            try:
                self.bus = smbus2.SMBus(self.i2c_bus)
                self.get_logger().info(f"BMI085 I2C Bus {self.i2c_bus} uzerinden baslatildi.")
                # Uyandırma komutları (Gerekirse datasheet'e gore eklenebilir)
                # Accel Normal Mode (0x7D -> 0x04)
                self.bus.write_byte_data(self.accel_addr, 0x7D, 0x04)
                time.sleep(0.05)
            except Exception as e:
                self.get_logger().error(f"I2C Baslatma hatasi: {e}")
                self.bus = None

        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

    def read_word(self, addr, reg):
        try:
            # LSB, MSB okuma (Little Endian)
            low = self.bus.read_byte_data(addr, reg)
            high = self.bus.read_byte_data(addr, reg + 1)
            val = (high << 8) + low
            if val >= 0x8000:
                return -((65535 - val) + 1)
            else:
                return val
        except Exception:
            return 0

    def timer_callback(self):
        if self.bus is None:
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        try:
            # İvmeölçer Verileri (Reg: 0x12 X, 0x14 Y, 0x16 Z)
            accel_x = self.read_word(self.accel_addr, 0x12)
            accel_y = self.read_word(self.accel_addr, 0x14)
            accel_z = self.read_word(self.accel_addr, 0x16)

            # Cayroskop Verileri (Reg: 0x02 X, 0x04 Y, 0x06 Z)
            gyro_x = self.read_word(self.gyro_addr, 0x02)
            gyro_y = self.read_word(self.gyro_addr, 0x04)
            gyro_z = self.read_word(self.gyro_addr, 0x06)

            # LSB'den fiziksel birime donusturme (Varsayilan +/-3G ve +/-2000 dps araligi)
            # ROS standartlarina gore (m/s^2 ve rad/s) - kalibrasyon carpani eklenebilir
            accel_scale = (3.0 * 9.81) / 32768.0
            gyro_scale = (2000.0 * (math.pi / 180.0)) / 32768.0

            msg.linear_acceleration.x = accel_x * accel_scale
            msg.linear_acceleration.y = accel_y * accel_scale
            msg.linear_acceleration.z = accel_z * accel_scale

            msg.angular_velocity.x = gyro_x * gyro_scale
            msg.angular_velocity.y = gyro_y * gyro_scale
            msg.angular_velocity.z = gyro_z * gyro_scale

            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Sensorden veri okunamadi: {e}", throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = BMI085Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
