#!/bin/bash

# Nova Engine V3 - Arkadaşınız için Derleme Scripti
echo "=== Nova Engine V3 - Arkadaşınız için Derleme ==="

# Gerekli paketleri kontrol et
echo "Gerekli paketler kontrol ediliyor..."
if ! pkg-config --exists opencv4; then
    echo "OpenCV4 bulunamadı! Yükleyin: sudo apt install libopencv-dev"
    exit 1
fi

if ! pkg-config --exists libavcodec; then
    echo "FFmpeg bulunamadı! Yükleyin: sudo apt install ffmpeg libavcodec-dev"
    exit 1
fi

echo "Tüm bağımlılıklar mevcut!"

# Build klasörü oluştur
mkdir -p build
cd build

# CMake ile derle
echo "CMake ile derleniyor..."
cmake .. -DCMAKE_BUILD_TYPE=Release

# Sadece çalışan uygulamaları derle
echo "Video chat derleniyor..."
make video_chat -j$(nproc)

echo "UDP chat derleniyor..."
make udp_chat -j$(nproc)

echo ""
echo "=== Derleme Tamamlandı! ==="
echo "Çalıştırılabilir dosyalar:"
echo "- ./video_chat (Görüntülü iletişim)"
echo "- ./udp_chat (Mesajlaşma)"
echo ""
echo "Kullanım:"
echo "./video_chat  # Görüntülü chat için"
echo "./udp_chat    # Mesajlaşma için" 