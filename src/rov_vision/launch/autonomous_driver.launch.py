"""
ROS2 Launch Dosyası - Otonom Sürüş V2 (Yarışma Modu)
Kullanım: ros2 launch rov_vision autonomous_driver.launch.py

Bu launch dosyası:
1. video_publisher     → USB kameradan görüntü yayınlar
2. distance_publisher  → Akustik mesafe sensöründen mesafe yayınlar
3. autonomous_driver   → Çizgi takip + Mesafe kontrolü + Serial PWM çıkışı

Yarışma Günü Hızlı Ayarlar:
  - HSV renk eşikleri → hsv_lower / hsv_upper
  - Hedef mesafe       → target_distance_cm
  - İleri hız          → linear_speed
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
    video_source_arg = DeclareLaunchArgument(
        'video_source', default_value='0',
        description='Kamera ID (0, 1) veya video dosya yolu (.mp4)'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='COM8',
        description='Arduino/STM32 serial portu (motor kontrolcü)'
    )
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate', default_value='115200',
        description='Motor kontrolcü serial baud rate'
    )
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed', default_value='0.15',
        description='İleri hız (m/s) — yarışmada 0.10-0.20 arası dene'
    )
    distance_serial_port_arg = DeclareLaunchArgument(
        'distance_serial_port', default_value='COM9',
        description='Mesafe sensörü UART portu'
    )
    distance_baud_rate_arg = DeclareLaunchArgument(
        'distance_baud_rate', default_value='9600',
        description='Mesafe sensörü baud rate'
    )
    target_distance_arg = DeclareLaunchArgument(
        'target_distance_cm', default_value='25.0',
        description='Tahtadan hedef mesafe (cm) — havuzda kalibre et'
    )

    # ── Video Publisher Node ───────────────────────────────────────────────
    video_publisher_node = Node(
        package='rov_vision',
        executable='video_publisher',
        name='video_publisher',
        output='screen',
        parameters=[{
            'video_source': LaunchConfiguration('video_source')
        }]
    )

    # ── Distance Publisher Node (✅ YENİ) ──────────────────────────────────
    distance_publisher_node = Node(
        package='rov_vision',
        executable='distance_publisher',
        name='distance_publisher',
        output='screen',
        parameters=[{
            'serial_port':       LaunchConfiguration('distance_serial_port'),
            'baud_rate':         9600,
            'publish_topic':     '/distance_sensor',
            'protocol':          'binary',      # Akustik sensör binary protokol
            'publish_rate_hz':   10.0,
            'min_range_cm':      8.0,
            'max_range_cm':      300.0,
            'median_filter_size': 5,
        }]
    )

    # ── Autonomous Driver Node (V2 — Yarışma Modu) ─────────────────────────
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
            'distance_topic': '/distance_sensor',   # ✅ YENİ

            # ── Hız Ayarları ───────────────────────────────────────────
            'linear_speed':       LaunchConfiguration('linear_speed'),
            'max_angular_z':      0.8,
            'max_vertical_speed': 0.3,              # ✅ YENİ

            # ── PID - Yanal Hata ───────────────────────────────────────
            'pid_kp':   0.003,
            'pid_ki':   0.0001,
            'pid_kd':   0.001,
            'pid_integral_limit': 100.0,

            # ── PID - Açı Düzeltme ─────────────────────────────────────
            'angle_kp': 0.005,
            'angle_kd': 0.001,

            # ── ✅ YENİ: PID - Mesafe Kontrolü (Dikey Eksen) ──────────
            'distance_pid_kp':             0.008,
            'distance_pid_ki':             0.001,
            'distance_pid_kd':             0.003,
            'distance_pid_integral_limit': 50.0,
            'target_distance_cm':          LaunchConfiguration('target_distance_cm'),
            'critical_distance_cm':        10.0,    # Bu altında ACİL!
            'max_safe_distance_cm':        100.0,
            'invert_vertical':             False,   # Dikey yön ters ise True yap

            # ── Görüntü İşleme ─────────────────────────────────────────
            'temporal_buffer_size': 5,
            'roi_top_ratio':       0.4,
            'min_contour_area':    500,
            'min_aspect_ratio':    1.5,    # Şerit en-boy oranı filtresi

            # ── HSV Renk Eşikleri ──────────────────────────────────────
            # Siyah şerit (kırmızı tahta üzerinde)
            # Yarışma günü havuzda kalibre et!
            'hsv_lower': [0,   0,   0],
            'hsv_upper': [180, 80,  60],

            # ── Çizgi Kayıp Toleransı ──────────────────────────────────
            'max_lost_frames':  30,
            'search_angular_z': 0.3,

            # ── ✅ YENİ: Hat Sonu Algılama ─────────────────────────────
            'end_of_line_lost_frames':  20,     # 20 frame kayıp → hat sonu
            'min_following_before_eol': 60,     # En az 60 frame takip etmiş ol
            'eol_stabilize_seconds':    0.5,    # 0.5 sn bekle → MISSION_READY (anında!)

            # ── ✅ YENİ: Acil Durum ────────────────────────────────────
            'emergency_pullback_frames': 10,
        }]
    )

    return LaunchDescription([
        camera_topic_arg,
        video_source_arg,
        serial_port_arg,
        baud_rate_arg,
        linear_speed_arg,
        distance_serial_port_arg,
        distance_baud_rate_arg,
        target_distance_arg,
        video_publisher_node,
        distance_publisher_node,
        autonomous_driver_node,
    ])
