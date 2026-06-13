"""
ROS2 Launch Dosyası - Otonom Sürüş (Çizgi Takip → Serial PWM)
Kullanım: ros2 launch rov_vision autonomous_driver.launch.py

Bu launch dosyası:
1. video_publisher  → USB kameradan görüntü yayınlar
2. autonomous_driver → Çizgi takip + Serial PWM çıkışı

Manuel sürüşteki serial protokolüyle birebir aynı çıkış verir.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ── Launch Argümanları ──────────────────────────────────────────────────
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic', default_value='/camera/image_raw',
        description='Kamera görüntü topic\'i'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='COM8',
        description='Arduino/STM32 serial portu (manuel sürüşle aynı)'
    )
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate', default_value='115200',
        description='Serial baud rate (manuel sürüşle aynı)'
    )
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed', default_value='0.15',
        description='İleri hız (m/s)'
    )

    # ── Video Publisher Node ───────────────────────────────────────────────
    video_publisher_node = Node(
        package='rov_vision',
        executable='video_publisher',
        name='video_publisher',
        output='screen',
    )

    # ── Autonomous Driver Node ─────────────────────────────────────────────
    autonomous_driver_node = Node(
        package='rov_vision',
        executable='autonomous_driver',
        name='autonomous_driver',
        output='screen',
        parameters=[{
            # ── Serial (Manuel sürüşle aynı) ───────────────────────────
            'serial_port':    LaunchConfiguration('serial_port'),
            'baud_rate':      115200,

            # ── Topic Ayarları ─────────────────────────────────────────
            'camera_topic':   LaunchConfiguration('camera_topic'),
            'cmd_vel_topic':  '/cmd_vel',
            'depth_topic':    '/depth_sensor',

            # ── Hız Ayarları ───────────────────────────────────────────
            'linear_speed':   LaunchConfiguration('linear_speed'),
            'max_angular_z':  0.8,

            # ── PID - Yanal Hata ───────────────────────────────────────
            'pid_kp':   0.003,
            'pid_ki':   0.0001,
            'pid_kd':   0.001,
            'pid_integral_limit': 100.0,

            # ── PID - Açı Düzeltme ─────────────────────────────────────
            'angle_kp': 0.005,
            'angle_kd': 0.001,

            # ── Görüntü İşleme ─────────────────────────────────────────
            'temporal_buffer_size': 5,
            'roi_top_ratio':        0.4,
            'min_contour_area':     500,

            # ── HSV Renk Eşikleri ──────────────────────────────────────
            'hsv_lower': [0,   0,   0],
            'hsv_upper': [180, 255, 255],

            # ── Çizgi Kayıp Toleransı ──────────────────────────────────
            'max_lost_frames':  30,
            'search_angular_z': 0.3,

            # ── Derinlik ───────────────────────────────────────────────
            'target_depth': 1.0,
        }]
    )

    return LaunchDescription([
        camera_topic_arg,
        serial_port_arg,
        baud_rate_arg,
        linear_speed_arg,
        video_publisher_node,
        autonomous_driver_node,
    ])
