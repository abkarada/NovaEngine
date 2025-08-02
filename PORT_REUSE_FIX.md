# NovaEngine Port Reuse Fix ✅

## Problem Solved
**Issue**: Sender ve receiver aynı portları kullanamıyordu, bağlantı kurulamıyordu.

**Root Cause**: 
- Sender ve receiver ayrı socket'ler oluşturuyordu
- `SO_REUSEADDR` ve `SO_REUSEPORT` doğru ayarlanmamıştı
- Port çakışması yaşanıyordu

## Solution: Global Socket Management

### 1. Global Socket System
- Tek bir socket pool'u hem sender hem receiver tarafından kullanılıyor
- `SO_REUSEADDR` ve `SO_REUSEPORT` doğru ayarlandı
- Port çakışması ortadan kalktı

### 2. Updated Architecture
```
┌─────────────────┐    ┌─────────────────┐
│   Sender        │    │   Receiver      │
│                 │    │                 │
│  ┌───────────┐  │    │  ┌───────────┐  │
│  │Global     │  │    │  │Global     │  │
│  │Sockets    │◄─┼────┼─►│Sockets    │  │
│  └───────────┘  │    │  └───────────┘  │
└─────────────────┘    └─────────────────┘
         │                       │
         └───────┬───────────────┘
                 │
         ┌───────▼───────┐
         │   UDP Ports   │
         │  8080,8081,   │
         │  8082...      │
         └───────────────┘
```

## Updated Usage

### 1. Both Mode (Recommended)
```bash
# Aynı makinede hem gönder hem al
./bin/novaengine both 127.0.0.1 8080 8081 8082
```

### 2. Sender Only
```bash
# Sadece gönder
./bin/novaengine sender 192.168.1.100 8080 8081 8082
```

### 3. Receiver Only
```bash
# Sadece al
./bin/novaengine receiver 8080 8081 8082
```

## Key Improvements

### 1. Socket Options
```cpp
// Port reuse için gerekli ayarlar
setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
#ifdef SO_REUSEPORT
setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
#endif
```

### 2. Global Socket Management
```cpp
// Global socket pool
static std::vector<int> global_sockets;
static bool global_sockets_initialized = false;

// Hem sender hem receiver kullanabilir
bool init_global_sockets(const std::vector<int>& ports);
std::vector<int> get_global_sockets();
void close_global_sockets();
```

### 3. Proper Cleanup
```cpp
// Program sonunda global socket'leri temizle
close_global_sockets();
```

## Testing Instructions

### Local Test
```bash
# Terminal 1: Both mode
./bin/novaengine both 127.0.0.1 8080 8081 8082

# Terminal 2: Receiver only (farklı makine için)
./bin/novaengine receiver 8080 8081 8082
```

### Network Test
```bash
# Makine A (Sender)
./bin/novaengine sender 192.168.1.100 8080 8081 8082

# Makine B (Receiver)  
./bin/novaengine receiver 8080 8081 8082
```

## Expected Behavior

### Console Output
```
[main] Starting NovaEngine in both mode with 3 ports
[global_sockets] Socket bound to port 8080
[global_sockets] Socket bound to port 8081
[global_sockets] Socket bound to port 8082
[global_sockets] ✅ 3 UDP sockets prepared for send/receive.
[receiver] Using 3 global sockets for receiving...
Receiver started with 3 sockets...
[ADAPTIVE] Bitrate: 600k -> 1000k, RTT: 45ms, Loss: 2.1%
```

### Performance
- ✅ **Port Conflict**: Çözüldü
- ✅ **Connection**: Başarılı
- ✅ **Bidirectional**: Hem gönder hem al
- ✅ **Multipath**: 3 port üzerinden paralel

## Troubleshooting

### Port Already in Use
```bash
# Port'ları kontrol et
sudo netstat -tulpn | grep :8080

# Eğer kullanımdaysa farklı port'lar dene
./bin/novaengine both 127.0.0.1 9090 9091 9092
```

### Permission Denied
```bash
# Port 1024'ten küçük portlar için sudo gerekebilir
sudo ./bin/novaengine both 127.0.0.1 80 81 82
```

### Firewall Issues
```bash
# UDP port'larını aç
sudo ufw allow 8080/udp
sudo ufw allow 8081/udp  
sudo ufw allow 8082/udp
```

## Next Steps
1. ✅ Port reuse sorunu çözüldü
2. ✅ Global socket management eklendi
3. ✅ Both mode ile test edildi
4. 🔄 Network testleri yapılacak
5. 🔄 Performance ölçümleri

NovaEngine artık tek porttan hem gönderme hem dinleme yapabiliyor! 🚀 