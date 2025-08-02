#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include <cstdint>
#include <memory>

// FFmpeg forward declarations
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
#include <libswscale/swscale.h>
}

// NovaEngine Ultra Stream - Dynamic CPU-Based Encoder
// Netflix-quality compression with real-time bitrate adaptation

class FFmpegEncoder {
public:
    FFmpegEncoder(int width = 1280, int height = 720, int fps = 30, int bitrate = 3000);
    ~FFmpegEncoder();
    
    // Encode OpenCV frame to H.264
    bool encodeFrame(const cv::Mat& bgrFrame, std::vector<uint8_t>& outEncodedData);
    
    // Dynamic bitrate adaptation based on network conditions
    void setBitrate(int bitrate_kbps);
    
    // Set frame rate dynamically
    void setFrameRate(int fps);
    
    // Set resolution dynamically
    void setResolution(int width, int height);
    
    // Calculate scene complexity for adaptive encoding
    double calculateSceneComplexity(const cv::Mat& frame);
    
    // Get encoder statistics
    struct EncoderStats {
        uint32_t frames_encoded;
        uint32_t total_bytes;
        double avg_bitrate_kbps;
        double avg_fps;
        double avg_complexity;
        uint64_t total_encode_time_us;
        int current_bitrate;
        int current_fps;
        int current_width;
        int current_height;
    };
    
    EncoderStats getStats() const;
    
    // Adaptive bitrate control
    void adaptBitrate(double network_kbps, double loss_ratio, double rtt_ms);
    
    // Get optimal bitrate based on network conditions
    int calculateOptimalBitrate(double network_kbps, double loss_ratio, double rtt_ms) const;
    
    // Get optimal FPS based on network conditions
    int calculateOptimalFPS(double network_kbps, double loss_ratio) const;
    
    // Get optimal encoder parameters
    struct EncoderParams {
        int bitrate_kbps;
        int fps;
        int crf;
        std::string preset;
        int keyint;
        int refs;
        bool low_latency;
    };
    
    EncoderParams calculateOptimalParams(double network_kbps, double loss_ratio, double rtt_ms) const;
    
    // Get current encoding parameters
    int getCurrentBitrate() const { return current_bitrate_; }
    int getCurrentFPS() const { return current_fps_; }
    std::pair<int, int> getCurrentResolution() const { return {current_width_, current_height_}; }

private:
    // FFmpeg components
    const AVCodec* codec_;
    AVCodecContext* codecContext_;
    AVFrame* frame_;
    AVPacket* pkt_;
    SwsContext* swsCtx_;
    
    // Encoding parameters
    int current_width_;
    int current_height_;
    int current_fps_;
    int current_bitrate_;
    
    // Statistics
    EncoderStats stats_;
    
    // Initialize encoder with optimal settings
    void initEncoder();
    
    // Configure encoder for low latency
    void configureLowLatency();
    
    // Update encoder parameters
    void updateEncoderParams();
    
    // Convert OpenCV frame to FFmpeg format
    bool convertFrame(const cv::Mat& bgrFrame);
    
    // Clean up FFmpeg resources
    void cleanup();
    
    // Calculate optimal bitrate based on network conditions
    int calculateOptimalBitrate(double network_kbps, double loss_ratio) const;
    
    // Calculate optimal FPS based on network conditions
    int calculateOptimalFPS(double network_kbps) const;
    
    // Calculate optimal resolution based on network conditions
    std::pair<int, int> calculateOptimalResolution(double network_kbps) const;
};

// Global encoder interface
extern FFmpegEncoder* g_encoder;

void initEncoder(int width = 1280, int height = 720, int fps = 30, int bitrate = 3000);
bool encodeFrame(const cv::Mat& frame, std::vector<uint8_t>& data);
void setEncoderBitrate(int bitrate_kbps);
void adaptEncoderToNetwork(double network_kbps, double loss_ratio, double rtt_ms);
void shutdownEncoder();
