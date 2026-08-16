#!/bin/bash
# =====================================================================
# JETSON DOCKER ÇALIŞTIRMA SCRİPTİ (DONANIM HIZLANDIRMA AKTİF)
# =====================================================================

echo "🚀 Jetson ROS2 Docker Başlatılıyor... Kasma sorunu çözülüyor!"

# xhost ile GUI erişimini aç (Ekranda kamera görüntüsünü görebilmek için)
xhost +local:root

# İnternetsiz havuz ortamı için İLK SEFERE MAHSUS build al (veya var olanı kullan)
if [[ "$(sudo docker images -q su-alti-rov-env 2> /dev/null)" == "" ]]; then
    echo "⚠️  İlk kurulum yapılıyor... Bu işlem SADECE BİR KERE yapılacak."
    echo "⬇️  İnternet bağlantınızın olduğundan emin olun, sensör kütüphaneleri indiriliyor..."
    sudo docker build -t su-alti-rov-env -f Dockerfile.jetson .
    echo "✅ Kurulum tamamlandı! Artık havuzda İNTERNETSİZ çalışabilirsin."
fi

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
    su-alti-rov-env \
    /bin/bash -c "source /opt/ros/\$ROS_DISTRO/setup.bash && echo '✅ Docker Hazır! Şimdi colcon build ve source edip projeyi başlatabilirsin.' && /bin/bash"
