"""
ROS2 Launch Dosyası - Su Altı Çizgi Takip
Kullanım: ros2 launch <paket> launch_line_follower.py
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
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='/cmd_vel',
        description='Hız komutu topic\'i'
    )
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed', default_value='0.15',
        description='İleri hız (m/s)'
    )

    # ── Ana Node ────────────────────────────────────────────────────────────
    # ── Ana Node ────────────────────────────────────────────────────────────
    line_follower_node = Node(
        package='rov_vision',           # BURAYI BİZİM PAKET ADIYLA DEĞİŞTİRDİK
        executable='line_follower',     # BURAYI BİZİM setup.py'DAKİ İSİMLE DEĞİŞTİRDİK
        name='underwater_line_follower',
        output='screen',
        parameters=[{
            # ── Topic Ayarları ──────────────────────────────────────────
            'camera_topic':   LaunchConfiguration('camera_topic'),
            'cmd_vel_topic':  LaunchConfiguration('cmd_vel_topic'),
            'depth_topic':    '/depth_sensor',

            # ── Hız Ayarları ────────────────────────────────────────────
            'linear_speed':   LaunchConfiguration('linear_speed'),
            'max_angular_z':  0.8,

            # ── PID - Yanal Hata ────────────────────────────────────────
            # Su altı: daha yavaş ve kararlı → düşük Kp, biraz Ki
            'pid_kp':   0.003,
            'pid_ki':   0.0001,
            'pid_kd':   0.001,
            'pid_integral_limit': 100.0,

            # ── PID - Açı Düzeltme ──────────────────────────────────────
            'angle_kp': 0.005,
            'angle_kd': 0.001,

            # ── Görüntü İşleme ──────────────────────────────────────────
            'temporal_buffer_size': 5,   # Zamansal ortalama kare sayısı
            'roi_top_ratio':        0.4, # Görüntünün üst %40'ı atılır
            'min_contour_area':     500, # px² altı konturlar gürültü sayılır

            # ── HSV Renk Eşikleri ───────────────────────────────────────
            # SIYAH çizgi (beton/demir boru):
            'hsv_lower': [0,   0,   0],
            'hsv_upper': [180, 255,  255],
            # SARI çizgi (deniz tabanı sarı boru/kablo) için:
            # 'hsv_lower': [20,  80,  80],
            # 'hsv_upper': [40, 255, 255],
            # TURUNCU çizgi için:
            # 'hsv_lower': [5,  100, 100],
            # 'hsv_upper': [25, 255, 255],

            # ── Çizgi Kayıp Toleransı ───────────────────────────────────
            'max_lost_frames':  30,   # Bu kare sayısı sonrası LOST durumu
            'search_angular_z': 0.3, # Arama modunda dönüş hızı (rad/s)

            # ── Derinlik ────────────────────────────────────────────────
            'target_depth': 1.0,     # Hedef derinlik (metre)
        }]
    )

    return LaunchDescription([
        camera_topic_arg,
        cmd_vel_topic_arg,
        linear_speed_arg,
        line_follower_node,
    ])
