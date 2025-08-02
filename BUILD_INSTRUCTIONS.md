# NovaEngine Ultra Stream - Build Instructions

## 🔧 **Son Güncellemeler**

### **Panel Boyutları Düzeltildi**
- **640x640 (Square Mode)**: Hem sender hem receiver
- **Kamera Erişimi**: Otomatik kamera algılama
- **Test Pattern**: Kamera yoksa fallback

### **Performans İyileştirmeleri**
- **Boyut Sabitliği**: Frame reconstruction'da tutarlı boyut
- **Decoder Düzeltmeleri**: H.264 decode hataları giderildi
- **Copy Optimizasyonu**: Const reference kullanımı

## 🚀 **Derleme Komutları**

```bash
# Kamera erişimi için kullanıcıyı video grubuna ekle
sudo usermod -a -G video $USER
newgrp video

# Temiz derleme
cd /home/ryuzaki/Desktop/NovaEngine
rm -rf build && mkdir build && cd build
cmake .. && make -j$(nproc)

# Test
cd .. && ./bin/novaengine sender 127.0.0.1 5001 5002
```

## 📋 **Gereksinimler**

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    libopencv-dev \
    libavcodec-dev \
    libavutil-dev \
    libswscale-dev \
    libavformat-dev \
    libswresample-dev \
    libjerasure-dev \
    pkg-config

# Kullanıcıyı video grubuna ekle
sudo usermod -a -G video $USER
newgrp video
```

## ✅ **Test Komutları**

```bash
# Sender modu (kamera + test pattern)
./bin/novaengine sender 127.0.0.1 5001 5002

# Receiver modu (720x1280 panel)
./bin/novaengine receiver 127.0.0.1 5001 5002

# Both modu (hem gönder hem al)
./bin/novaengine both 127.0.0.1 5001 5002
```

## 🎯 **Özellikler**

✅ **640x640 Panel**: Square mode optimizasyonu  
✅ **Kamera Erişimi**: Otomatik kamera algılama  
✅ **Multi-tunnel UDP**: 3 UDP portu üzerinden veri akışı  
✅ **MTU-aware slicing**: 1000 byte chunk'lar  
✅ **Reed-Solomon FEC**: k=6, r=2 parity  
✅ **Adaptive scheduler**: RTT/loss tabanlı path seçimi  
✅ **Dynamic bitrate**: 4Mbps@30fps  
✅ **Time-windowed jitter buffer**: <50ms latency  
✅ **Zero-copy I/O**: std::move optimizasyonu  
✅ **Boyut Sabitliği**: Tutarlı frame reconstruction  

## 🔍 **Sorun Giderme**

### **Kamera Erişimi Sorunu**
```bash
# Kullanıcıyı video grubuna ekle
sudo usermod -a -G video $USER
newgrp video

# Veya sistemi yeniden başlat
sudo reboot
```

### **Panel Boyutları**
- **Sender**: 640x640 (square)
- **Receiver**: 640x640 (square)
- **Test Pattern**: 640x640 (square)

### **Decoder Hataları**
- H.264 decode hataları düzeltildi
- Frame size mismatch kontrolü eklendi
- Error handling iyileştirildi

## 📊 **Performans Hedefleri**

- **Latency**: <50ms
- **Reliability**: Zero packet loss (FEC ile)
- **Quality**: 4Mbps@30fps (640x640)
- **Throughput**: 20+ Mbps
- **Panel Boyutu**: 640x640 (tutarlı) 