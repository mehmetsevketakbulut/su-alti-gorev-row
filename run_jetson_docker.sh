#!/bin/bash
# =====================================================================
# JETSON DOCKER ÇALIŞTIRMA SCRİPTİ (DONANIM HIZLANDIRMA AKTİF)
# =====================================================================
# Yarışmada panik yapmamak için kamerayı, sensörleri (I2C) ve 
# motorları (USB) otomatik olarak ekran kartıyla (GPU) Docker'a bağlar.

# 📌 DİKKAT: ROS Sürümüne göre burayı değiştir:
# Eğer Jetson Nano kullanıyorsanız (Jetpack 4.6) -> ROS Foxy:
IMAGE_NAME="dustynv/ros:foxy-desktop-l4t-r32.7.1"
# Eğer Jetson Orin/Xavier kullanıyorsanız (Jetpack 5) -> ROS Humble:
# IMAGE_NAME="dustynv/ros:humble-desktop-l4t-r35.2.1"

echo "🚀 Jetson ROS2 Docker Başlatılıyor... Kasma sorunu çözülüyor!"

# xhost ile GUI erişimini aç (Ekranda kamera görüntüsünü görebilmek için)
xhost +local:root

# Docker'ı donanım erişimleriyle başlat
sudo docker run -it --rm \
    --net=host \
    --runtime nvidia \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    -v $(pwd):/workspace/su-alti-gorev-row \
    --device /dev/video0 \
    --device /dev/i2c-1 \
    --device /dev/i2c-7 \
    --device /dev/ttyUSB0 \
    --device /dev/ttyTHS1 \
    --privileged \
    $IMAGE_NAME \
    /bin/bash -c "cd /workspace/su-alti-gorev-row && source /opt/ros/\$ROS_DISTRO/setup.bash && echo '⚙️ Gerekli Python kütüphaneleri (Sensörler için) kuruluyor...' && pip3 install pyserial adafruit-circuitpython-bno08x ms5837 && echo '✅ Docker Hazır! Şimdi colcon build ve source edip projeyi başlatabilirsin.' && /bin/bash"
