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
        self.declare_parameter('i2c_bus', 0)
        self.declare_parameter('publish_topic', '/depth_sensor')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('fluid_density', 'saltwater')

        # Parametre değerlerini al
        raw_bus = self.get_parameter('i2c_bus').value
        self.i2c_bus = int(raw_bus)  # Kesinlikle integer'a cevir
        self.publish_topic = self.get_parameter('publish_topic').value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.fluid_density_str = self.get_parameter('fluid_density').value

        self.get_logger().info(f"---- MS5837 BASLATILIYOR ----")
        self.get_logger().info(f"Hedef I2C Bus: {self.i2c_bus} (Tipi: {type(self.i2c_bus)})")

        # Publisher'ı oluştur
        self.publisher_ = self.create_publisher(Float32, self.publish_topic, 10)

        # MS5837 sensörünü başlat (Jetson'da timeout olabildiği için retry döngüsü)
        self.sensor = ms5837.MS5837_02BA(self.i2c_bus)
        
        self.sensor_initialized = False
        for attempt in range(5):
            try:
                if self.sensor.init():
                    self.sensor_initialized = True
                    break
                else:
                    self.get_logger().warn(f"MS5837 init() False dondu (Deneme {attempt+1})")
            except Exception as e:
                self.get_logger().warn(f"MS5837 başlatma denemesi {attempt+1}/5 başarısız (Exception): {e}")
            time.sleep(0.5)

        if not self.sensor_initialized:
            self.get_logger().error("MS5837 sensörü başlatılamadı! Lütfen I2C bağlantısını kontrol edin.")
        else:
            self.get_logger().info(f"MS5837 sensörü I2C-{self.i2c_bus} üzerinde başarıyla başlatıldı.")
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
        
        # Ek Topic'ler: Basinc (mBar) ve Sicaklik (C)
        self.pub_press = self.create_publisher(Float32, '/pressure_sensor', 10)
        self.pub_temp = self.create_publisher(Float32, '/temperature_sensor', 10)

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
                pressure_mbar = self.sensor.pressure()
                temp_c = self.sensor.temperature()

                # Mesajları oluştur ve yayınla
                msg_depth = Float32(); msg_depth.data = float(depth_cm)
                self.publisher_.publish(msg_depth)
                
                msg_press = Float32(); msg_press.data = float(pressure_mbar)
                self.pub_press.publish(msg_press)
                
                msg_temp = Float32(); msg_temp.data = float(temp_c)
                self.pub_temp.publish(msg_temp)
                
                self.get_logger().debug(f"Derinlik: {depth_cm:.2f}cm | Basinc: {pressure_mbar:.2f}mBar | Sicaklik: {temp_c:.2f}C")
            else:
                self.get_logger().error("Sensörden veri okunamadı!", throttle_duration_sec=2.0)
        except Exception as e:
            # I2C hatlarinda (ozellikle Jetson'da) anlik Errno 121 olmasi normaldir. Node'u cokertmesin.
            self.get_logger().warn(f"I2C Anlik Okuma Hatasi (Gormezden gelinebilir): {str(e)}", throttle_duration_sec=2.0)


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
