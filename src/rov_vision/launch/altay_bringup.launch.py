#!/usr/bin/env python3
"""
=============================================================================
ALTAY ROV - TEK KOMUT BASLATMA
=============================================================================
  ros2 launch altay_rov altay_bringup.launch.py

Parametre degistirmek icin:
  ros2 launch altay_rov altay_bringup.launch.py pc_ip:=192.168.1.7 \
       serial_port:=/dev/ttyUSB1

BASLATILAN NODE'LAR
-------------------
  YENI (bu paketle geliyor):
    rov_bridge      Seri portu acan TEK node + mod secici (mux)
    ground_link     PC ile UDP koprusu
    line_follower   Otonom cizgi takip (duzeltilmis aci sarmasi ile)

  MEVCUT (sizin calisan node'lariniz, dosyalarina DOKUNULMADI):
    video_publisher     USB yakalama karti -> /camera/image_raw
    imu_publisher       BNO085 -> /imu/data        (i2c_bus=7 ile)
    pressure_publisher  MS5837 -> /depth_sensor
    distance_publisher  DYP    -> /distance_sensor (Linux portu ile)
    ros2_video_streamer /camera/image_raw -> http://JETSON:5000/video

PARAMETRE DUZELTMELERI (dosya duzenlemeden, buradan geciliyor)
--------------------------------------------------------------
  imu_publisher      i2c_bus: 1 -> 7
      Varsayilan 1'di ama hardware_test.py ve imu_debug.py dosyalariniz
      BNO085'i I2C-7'de buluyor. Varsayilanla node sessizce bos veri yayinlar.

  distance_publisher serial_port: 'COM9' -> '/dev/ttyTHS1'
      Varsayilan bir Windows port adiydi, simdi Jetson donanimsal UART (Pin 8-10) olarak guncellendi.

  pressure_publisher DOSYADA tek satir degisiklik gerekiyor (asagiya bakin).
=============================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pc_ip = LaunchConfiguration('pc_ip')
    serial_port = LaunchConfiguration('serial_port')
    dyp_port = LaunchConfiguration('dyp_port')
    imu_bus = LaunchConfiguration('imu_bus')
    stream_topic = LaunchConfiguration('stream_topic')

    args = [
        DeclareLaunchArgument('pc_ip', default_value='192.168.1.50',
                              description='Yer istasyonu PC IP adresi'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='Ana ROV Deneyap USB portu'),
        DeclareLaunchArgument('dyp_port', default_value='/dev/ttyTHS1',
                              description='DYP mesafe sensoru portu (Pin 8-10)'),
        DeclareLaunchArgument('imu_bus', default_value='7',
                              description='BNO085 I2C bus numarasi'),
        DeclareLaunchArgument('stream_topic', default_value='/camera/image_raw',
                              description='PC yayini icin topic. Otonom kosuda '
                                          '/line_follower/debug_image yapabilirsiniz '
                                          'ama o node cokerse video da kesilir.'),
    ]

    nodes = [
        # ---------------- YENI NODE'LAR ----------------
        Node(package='rov_vision', executable='rov_bridge', name='rov_bridge',
             output='screen', emulate_tty=True,
             parameters=[{'serial_port': serial_port,
                          'baud_rate': 115200,
                          'send_rate_hz': 50.0,
                          'manual_timeout_s': 0.4}]),

        Node(package='rov_vision', executable='ground_link', name='ground_link',
             output='screen', emulate_tty=True,
             parameters=[{'pc_ip': pc_ip,
                          'listen_port': 5005,
                          'telemetry_port': 5006,
                          'telemetry_rate_hz': 10.0}]),

        Node(package='rov_vision', executable='line_follower', name='line_follower',
             output='screen', emulate_tty=True,
             parameters=[{'linear_speed': 0.35,
                          'max_yaw': 0.60,
                          'hsv_upper': [180, 255, 70],
                          'tilt_scan_min': 25,
                          'tilt_scan_max': 65,
                          'tilt_lock': 50}]),

        # ---------------- MEVCUT NODE'LAR ----------------
        Node(package='rov_vision', executable='video_publisher',
             name='video_publisher', output='screen',
             parameters=[{'video_source': '0'}]),

        Node(package='rov_vision', executable='imu_publisher',
             name='imu_publisher', output='screen',
             parameters=[{'i2c_bus': imu_bus,       # 1 -> 7 DUZELTMESI
                          'i2c_address': 0x4A,
                          'publish_rate_hz': 50.0}]),

        Node(package='rov_vision', executable='pressure_publisher',
             name='pressure_publisher', output='screen',
             parameters=[{'i2c_bus': 1,
                          'fluid_density': 'freshwater',
                          'publish_rate_hz': 10.0}]),

        Node(package='rov_vision', executable='distance_publisher',
             name='distance_publisher', output='screen',
             parameters=[{'serial_port': dyp_port,  # COM9 -> Linux DUZELTMESI
                          'baud_rate': 9600,
                          'protocol': 'binary'}]),

        Node(package='rov_vision', executable='ros2_video_streamer',
             name='ros2_video_streamer', output='screen',
             parameters=[{'image_topic': stream_topic}]),
    ]

    return LaunchDescription(args + nodes)
