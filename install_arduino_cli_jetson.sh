#!/bin/bash
# Jetson Orin Nano için Arduino CLI ve Deneyap/ESP32 Kurulum Scripti

echo "🚀 Arduino CLI indiriliyor..."
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
sudo mv bin/arduino-cli /usr/local/bin/

echo "⚙️ Arduino CLI yapılandırılıyor..."
arduino-cli config init

# ESP32 ve Deneyap Kart URL'lerini ekle
arduino-cli config set board_manager.additional_urls "https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json,https://raw.githubusercontent.com/deneyapkart/deneyapkart-arduino-core/master/package_deneyapkart_index.json"

echo "📦 Çekirdekler güncelleniyor ve ESP32/Deneyap kuruluyor..."
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli core install deneyapkart:esp32

echo "📚 Gerekli kütüphaneler kuruluyor..."
arduino-cli lib install "ESP32Servo"
arduino-cli lib install "mcp_can"
arduino-cli lib install "PID"

# Deneyap IMU kütüphanesi
mkdir -p ~/Arduino/libraries
cd ~/Arduino/libraries
if [ ! -d "deneyapkart-6-eksen-ataletsel-olcum-birimi-arduino-library" ]; then
    echo "Deneyap IMU kütüphanesi indiriliyor..."
    git clone https://github.com/deneyapkart/deneyapkart-6-eksen-ataletsel-olcum-birimi-arduino-library.git
fi

echo "✅ Kurulum tamamlandı!"
echo "Lütfen şu komutu çalıştırıp Jetson'ı YENİDEN BAŞLATIN (USB yetkisi için):"
echo "sudo usermod -a -G dialout \$USER"
