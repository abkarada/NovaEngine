# Nova Engine V3 - Görüntülü İletişim Sistemi

## 🎥 Özellikler

- ✅ **Gerçek zamanlı video görüntüleme**
- ✅ **Kamera yakalama (OpenCV)**
- ✅ **JPEG sıkıştırma (80% kalite)**
- ✅ **UDP üzerinden video streaming**
- ✅ **Manuel IP/Port girişi**
- ✅ **İki pencere: "Sizin Görüntünüz" ve "Karşı Taraf"**
- ✅ **ESC tuşu ile çıkış**

## 🚀 Hızlı Başlangıç

### 1. Derleme

```bash
# Kendi bilgisayarınızda
cd Nova_Engine_V3
mkdir -p build && cd build
cmake ..
make video_chat -j$(nproc)
make udp_chat -j$(nproc)
```

### 2. Arkadaşınız için

```bash
# Arkadaşınızın bilgisayarında
./build_friend.sh
```

## 📹 Video Chat Kullanımı

### Her iki taraf da aynı uygulamayı kullanır:

```bash
./video_chat
```

### Örnek Kullanım:

**Sizin Tarafınız (192.168.1.254):**
```
Kendi IP adresinizi girin: 192.168.1.254
Kendi port numaranızı girin (1-65535): 45001
Karşı tarafın IP adresini girin: 192.168.1.5
Karşı tarafın port numarasını girin (1-65535): 45000
```

**Arkadaşınızın Tarafı (192.168.1.5):**
```
Kendi IP adresinizi girin: 192.168.1.5
Kendi port numaranızı girin (1-65535): 45000
Karşı tarafın IP adresini girin: 192.168.1.254
Karşı tarafın port numarasını girin (1-65535): 45001
```

## 💬 UDP Chat Kullanımı

```bash
./udp_chat
```

## 🎯 Teknik Detaylar

- **Video Çözünürlüğü:** 640x480
- **FPS:** 30
- **Sıkıştırma:** JPEG 80% kalite
- **Protokol:** UDP
- **Buffer Boyutu:** 65KB (video frame'leri için)

## 🔧 Gereksinimler

- **OpenCV4** (`libopencv-dev`)
- **FFmpeg** (`ffmpeg libavcodec-dev`)
- **C++17** uyumlu derleyici
- **Linux** (Ubuntu/Debian)

## 📦 Kurulum

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install build-essential cmake pkg-config
sudo apt install libopencv-dev ffmpeg libavcodec-dev
```

## 🐛 Sorun Giderme

### Kamera açılmıyor:
```bash
# Kamera izinlerini kontrol edin
ls -la /dev/video*
# Gerekirse kullanıcıyı video grubuna ekleyin
sudo usermod -a -G video $USER
```

### Port bağlanamıyor:
- Farklı port numarası deneyin
- Firewall ayarlarını kontrol edin
- Aynı portu kullanmadığınızdan emin olun

## 📞 İletişim

Video chat başarıyla çalıştıktan sonra:
- **"Sizin Görüntünüz"** penceresi: Kendi kameranız
- **"Karşı Taraf"** penceresi: Arkadaşınızın görüntüsü
- **ESC tuşu** ile çıkış

## 🎉 Başarı!

Artık gerçek görüntülü iletişim kurabilirsiniz! 🎥✨
