#!/bin/bash
# Jetson Orin Nano'dan Deneyap'a Kod Yükleme Scripti

PORT="/dev/ttyUSB0"
# Eğer farklı porttaysa (örneğin USB1), komut satırından port verebilirsiniz:
# ./upload_firmware_jetson.sh /dev/ttyUSB1
if [ ! -z "$1" ]; then
    PORT=$1
fi

SKETCH_PATH="src/rov_vision/rov_vision/AnaROV_vehicle/AnaROV_video_mission.ino"
FQBN="deneyapkart:esp32:deneyapkart"

echo "=========================================================="
echo " 🚀 Deneyap'a Kod Yükleniyor... ($PORT)"
echo " FQBN  : $FQBN"
echo " Dosya : $SKETCH_PATH"
echo "=========================================================="

# 1. Derleme
echo "🔨 Derleniyor (Bu işlem Jetson'da ilk seferde 1-2 dakika sürebilir)..."
arduino-cli compile --fqbn $FQBN $SKETCH_PATH

if [ $? -ne 0 ]; then
    echo "❌ Derleme HATASI! Lütfen koddaki hataları düzeltin."
    exit 1
fi

# 2. Yükleme
echo "📤 Yükleniyor ($PORT)..."
arduino-cli upload -p $PORT --fqbn $FQBN $SKETCH_PATH

if [ $? -eq 0 ]; then
    echo "✅ BAŞARILI! Kod Deneyap'a yüklendi ve görev çalışmaya hazır."
else
    echo "❌ Yükleme HATASI! Kabloyu kontrol edin veya boot butonuna (Deneyap üzerinde) basılı tutun."
fi
