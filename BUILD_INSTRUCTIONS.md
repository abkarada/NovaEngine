# NovaEngine Ultra Stream - Build Instructions

## 🔧 **Düzeltilen Sorunlar**

### **1. Eksik Include'lar**
- `#include <ctime>` eklendi (time() fonksiyonu için)

### **2. Null Pointer Kontrolleri**
- Global değişkenler için null kontrolü eklendi
- `g_encoder`, `g_scheduler`, `g_collector`, `g_sender_receiver` kontrolleri

### **3. Gereksiz Dosyalar Silindi**
- `chunk_dispatcher.cpp/hpp` - Artık kullanılmıyor
- `loss_tracker.cpp/hpp` - Eski implementasyon
- `rtt_monitor.cpp/hpp` - Eski implementasyon  
- `ping_handler.cpp/hpp` - Eski implementasyon
- `ping_sender.cpp/hpp` - Eski implementasyon

### **4. Compiler Warning'leri Düzeltildi**
- FFmpeg encoder'da narrowing conversion düzeltildi
- `static_cast<int>(bgrFrame.step)` eklendi

### **5. Overflow Sorunları Düzeltildi**
- Scheduler'da weight overflow koruması eklendi
- Weight hesaplama 1-1000 aralığında sınırlandı

## 🚀 **Derleme Komutları**

```bash
# Temiz derleme
rm -rf build
mkdir build
cd build

# CMake konfigürasyonu
cmake ..

# Derleme
make -j$(nproc)

# Çalıştırma
cd ..
./bin/novaengine sender 127.0.0.1 5001 5002
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
# Sender modu (test pattern)
./bin/novaengine sender 127.0.0.1 5001 5002

# Receiver modu (başka terminal)
./bin/novaengine receiver 127.0.0.1 5001 5002

# Both modu (hem gönder hem al)
./bin/novaengine both 127.0.0.1 5001 5002
```

## 🎯 **Özellikler**

✅ **Multi-tunnel UDP**: 3 UDP portu üzerinden veri akışı  
✅ **MTU-aware slicing**: 1000 byte chunk'lar  
✅ **Reed-Solomon FEC**: k=6, r=2 parity  
✅ **Adaptive scheduler**: RTT/loss tabanlı path seçimi  
✅ **Dynamic bitrate**: Network koşullarına göre uyarlama  
✅ **Time-windowed jitter buffer**: <50ms latency  
✅ **Zero-copy I/O**: std::move optimizasyonu  
✅ **Test pattern mode**: Kamera erişimi olmadan test  

## 🔍 **Sorun Giderme**

### **Kamera Erişimi Sorunu**
```bash
# Kullanıcıyı video grubuna ekle
sudo usermod -a -G video $USER
newgrp video

# Veya sistemi yeniden başlat
sudo reboot
```

### **Jerasure Kütüphanesi Bulunamıyor**
```bash
sudo apt-get install libjerasure-dev
```

### **FFmpeg Kütüphaneleri Bulunamıyor**
```bash
sudo apt-get install libavcodec-dev libavutil-dev libswscale-dev libavformat-dev libswresample-dev
```

## 📊 **Performans Hedefleri**

- **Latency**: <100ms (Zoom'dan daha hızlı)
- **Reliability**: Zero packet loss (FEC ile)
- **Quality**: Netflix kalitesinde dinamik sıkıştırma
- **Throughput**: Multi-tunnel ile maksimum bant genişliği 