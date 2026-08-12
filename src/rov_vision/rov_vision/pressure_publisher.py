#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import ms5837
import time

class PressurePublisher(Node):
    def __init__(self):
        super().__init__('pressure_publisher')

        # ROS2 Parametrelerini tanımla
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('publish_topic', '/depth_sensor')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('fluid_density', 'freshwater')

        # Parametre değerlerini al
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.publish_topic = self.get_parameter('publish_topic').value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.fluid_density_str = self.get_parameter('fluid_density').value

        # Publisher'ı oluştur
        self.publisher_ = self.create_publisher(Float32, self.publish_topic, 10)

        # MS5837 sensörünü başlat
        self.sensor = ms5837.MS5837_30BA(self.i2c_bus)

        if not self.sensor.init():
            self.get_logger().error("MS5837 sensörü başlatılamadı! Lütfen I2C bağlantısını ve adresini kontrol edin.")
            self.sensor_initialized = False
        else:
            self.get_logger().info("MS5837 sensörü başarıyla başlatıldı.")
            self.sensor_initialized = True
            
            # Akışkan yoğunluğunu ayarla
            if self.fluid_density_str.lower() == 'saltwater':
                self.sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
                self.get_logger().info("Akışkan yoğunluğu: Tuzlu Su")
            else:
                self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
                self.get_logger().info("Akışkan yoğunluğu: Tatlı Su")

        # Zamanlayıcıyı oluştur (10 Hz için 0.1 saniye)
        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if not self.sensor_initialized:
            self.get_logger().warn("Sensör başlatılamadığı için veri okunamıyor.", throttle_duration_sec=5.0)
            return

        # Sensörden veri okumayı dene
        try:
            if self.sensor.read():
                # Derinliği metre cinsinden al ve santimetreye çevir
                depth_m = self.sensor.depth()
                depth_cm = depth_m * 100.0

                # Mesajı oluştur ve yayınla
                msg = Float32()
                msg.data = float(depth_cm)
                self.publisher_.publish(msg)
                
                self.get_logger().debug(f"Yayınlanan derinlik: {depth_cm:.2f} cm")
            else:
                self.get_logger().error("Sensörden veri okunamadı!")
        except Exception as e:
            self.get_logger().error(f"Veri okunurken hata oluştu: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    pressure_publisher = PressurePublisher()

    try:
        rclpy.spin(pressure_publisher)
    except KeyboardInterrupt:
        pressure_publisher.get_logger().info("Düğüm klavye kesmesi ile durduruldu.")
    except Exception as e:
        pressure_publisher.get_logger().error(f"Düğüm çalışırken hata oluştu: {str(e)}")
    finally:
        pressure_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
