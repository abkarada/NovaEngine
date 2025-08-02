# NovaEngine – Real-Time P2P Media Engine by NovaSafer

> 🎥 Ultra-low latency, native media engine for secure peer-to-peer communication.  
> Developed as the core transport layer for the NovaSafer platform (SafeRoom Project).

---

## 🚀 What is NovaEngine?

NovaEngine is a high-performance C++ media core designed for **real-time, serverless audio/video streaming**.  
It works directly over **UDP**, integrates with NAT traversal (STUN/Hole Punching), and ensures full-duplex, low-latency transmission for decentralized communication.

Built for:
- 🔐 Secure environments
- 🕸️ Serverless P2P systems
- ⚡ Real-time responsiveness (<50ms)

---

## 🧱 Architecture Overview

+-------------------+ UDP +-------------------+
| NovaEngine (Peer) | <-------------> | NovaEngine (Peer) |
+-------------------+ +-------------------+
▲ ▲
🎤🎥 Capture 🎤🎥 Capture
│ │
🔄 Encode & Packetize 🔄 Encode & Packetize
│ │
📡 Send UDP 📡 Send UDP
│ │
📥 Receive UDP 📥 Receive UDP
│ │
🧠 Depacketize & Decode 🧠 Depacketize & Decode
│ │
🔊🎬 Playback 🔊🎬 Playback


---

## 🗂️ Directory Structure

NovaEngine/
├── include/
│ ├── packet.hpp
│ ├── audio_capture.hpp
│ ├── video_capture.hpp
│ ├── udp_sender.hpp
│ └── udp_receiver.hpp
├── src/
│ ├── audio_capture.cpp
│ ├── video_capture.cpp
│ ├── udp_sender.cpp
│ ├── udp_receiver.cpp
│ └── NovaEngine.cpp
├── libs/
│ ├── libopus.a
│ └── libjpeg.a
├── CMakeLists.txt
└── README.md


---

## ⚙️ Build Instructions

### 🧰 Dependencies

- C++17 or later
- OpenCV
- PortAudio
- libopus
- libjpeg (or ffmpeg for H.264/VP8 if needed)

### 🏗️ Compile with CMake

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
🧪 Usage
Run the media engine with:

It will:

Capture video/audio

Encode and fragment

Send over UDP

Receive incoming media

Decode and play

🔐 Integration with NovaSafer (Java Signaling Layer)
NovaEngine is designed to work in sync with the Java-based NovaSafer signaling infrastructure.
The Java layer handles:

NAT traversal

Peer matching

IP:Port discovery

Session key exchange (if encryption is enabled)

NovaEngine assumes the peer's public IP and port are already known and directly streams media.

📌 Roadmap
 MJPEG / PCM UDP streaming

 Opus codec integration

 Adaptive bitrate & error concealment

 Forward Error Correction (FEC)

 DTLS-SRTP encryption

 Hardware acceleration support (NVENC / VAAPI)

🧠 Authors
NovaSafer Engineering Team
Project System Engineer: Abdurrahman Karadağ
Project Tech Depart:Resul Sarıkaya


