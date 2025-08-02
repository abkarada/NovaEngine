#include "ffmpeg_encoder.h"
#include <stdexcept>
#include <iostream>
#include <thread>
#include <opencv2/imgproc.hpp>
#include <chrono>

// FFmpeg includes
extern "C" {
#include <libavutil/opt.h>
}

// NovaEngine Ultra Stream - Dynamic CPU-Based Encoder
// Netflix-quality compression with real-time bitrate adaptation

// Global encoder instance
FFmpegEncoder* g_encoder = nullptr;

FFmpegEncoder::FFmpegEncoder(int width, int height, int fps, int bitrate)
    : current_width_(width), current_height_(height), current_fps_(fps), current_bitrate_(bitrate),
      codec_(nullptr), codecContext_(nullptr), frame_(nullptr), pkt_(nullptr), swsCtx_(nullptr) {
    
    // Initialize statistics
    stats_ = {};
    
    initEncoder();
    configureLowLatency();
    
    std::cout << "[Encoder] Initialized: " << width << "x" << height 
              << "@" << fps << "fps, " << bitrate << "kbps" << std::endl;
}

FFmpegEncoder::~FFmpegEncoder() {
    cleanup();
}

bool FFmpegEncoder::encodeFrame(const cv::Mat& bgrFrame, std::vector<uint8_t>& outEncodedData) {
    if (!codecContext_ || !frame_ || !pkt_ || bgrFrame.empty()) {
        return false;
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Calculate scene complexity and adjust encoding parameters
    double complexity = calculateSceneComplexity(bgrFrame);
    
    // Convert OpenCV frame to FFmpeg format
    if (!convertFrame(bgrFrame)) {
        return false;
    }
    
    // Encode frame
    int ret = avcodec_send_frame(codecContext_, frame_);
    if (ret < 0) {
        std::cerr << "[Encoder] Error sending frame to encoder" << std::endl;
        return false;
    }
    
    outEncodedData.clear();
    
    while (ret >= 0) {
        ret = avcodec_receive_packet(codecContext_, pkt_);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            break;
        } else if (ret < 0) {
            std::cerr << "[Encoder] Error receiving packet from encoder" << std::endl;
            return false;
        }
        
        // Copy packet data
        outEncodedData.insert(outEncodedData.end(), pkt_->data, pkt_->data + pkt_->size);
        av_packet_unref(pkt_);
    }
    
    // Update statistics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    stats_.frames_encoded++;
    stats_.total_bytes += outEncodedData.size();
    stats_.total_encode_time_us += duration.count();
    stats_.avg_complexity = (stats_.avg_complexity * (stats_.frames_encoded - 1) + complexity) / stats_.frames_encoded;
    
    // Update average bitrate
    if (stats_.frames_encoded > 0) {
        stats_.avg_bitrate_kbps = (stats_.total_bytes * 8.0 * current_fps_) / (stats_.frames_encoded * 1000.0);
    }
    
    std::cout << "[Encoder] Encoded frame: " << outEncodedData.size() << " bytes, "
              << "complexity: " << complexity << ", time: " << duration.count() << "μs" << std::endl;
    
    return !outEncodedData.empty();
}

void FFmpegEncoder::setBitrate(int bitrate_kbps) {
    if (bitrate_kbps <= 0) return;
    
    current_bitrate_ = bitrate_kbps;
    codecContext_->bit_rate = bitrate_kbps * 1000;  // Convert to bits per second
    
    // Update encoder parameters
    updateEncoderParams();
    
    std::cout << "[Encoder] Bitrate updated to " << bitrate_kbps << " kbps" << std::endl;
}

void FFmpegEncoder::setFrameRate(int fps) {
    if (fps <= 0) return;
    
    current_fps_ = fps;
    codecContext_->time_base = {1, fps};
    codecContext_->framerate = {fps, 1};
    
    updateEncoderParams();
    
    std::cout << "[Encoder] Frame rate updated to " << fps << " fps" << std::endl;
}

void FFmpegEncoder::setResolution(int width, int height) {
    if (width <= 0 || height <= 0) return;
    
    current_width_ = width;
    current_height_ = height;
    
    // Reinitialize encoder with new resolution
    cleanup();
    initEncoder();
    configureLowLatency();
    
    std::cout << "[Encoder] Resolution updated to " << width << "x" << height << std::endl;
}

double FFmpegEncoder::calculateSceneComplexity(const cv::Mat& frame) {
    if (frame.empty()) return 0.0;
    
    cv::Mat gray, edges;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    // Calculate edge density as complexity measure
    cv::Canny(gray, edges, 50, 150);
    double edge_ratio = cv::countNonZero(edges) / static_cast<double>(edges.total());
    
    // Calculate variance as another complexity measure
    cv::Scalar mean, stddev;
    cv::meanStdDev(gray, mean, stddev);
    double variance = stddev[0] * stddev[0];
    
    // Normalize and combine metrics
    double complexity = (edge_ratio * 1000.0 + variance / 100.0) / 2.0;
    return std::min(complexity, 1.0); // Normalize to [0,1]
}

FFmpegEncoder::EncoderStats FFmpegEncoder::getStats() const {
    EncoderStats stats = stats_;
    stats.current_bitrate = current_bitrate_;
    stats.current_fps = current_fps_;
    stats.current_width = current_width_;
    stats.current_height = current_height_;
    
    return stats;
}

void FFmpegEncoder::adaptBitrate(double network_kbps, double loss_ratio) {
    int optimal_bitrate = calculateOptimalBitrate(network_kbps, loss_ratio);
    int optimal_fps = calculateOptimalFPS(network_kbps);
    auto optimal_resolution = calculateOptimalResolution(network_kbps);
    
    // Apply adaptations
    if (optimal_bitrate != current_bitrate_) {
        setBitrate(optimal_bitrate);
    }
    
    if (optimal_fps != current_fps_) {
        setFrameRate(optimal_fps);
    }
    
    if (optimal_resolution.first != current_width_ || optimal_resolution.second != current_height_) {
        setResolution(optimal_resolution.first, optimal_resolution.second);
    }
    
    std::cout << "[Encoder] Adapted to network: " << network_kbps << " kbps, "
              << "loss: " << (loss_ratio * 100) << "%, "
              << "new settings: " << optimal_bitrate << "kbps@" << optimal_fps 
              << "fps " << optimal_resolution.first << "x" << optimal_resolution.second << std::endl;
}

void FFmpegEncoder::initEncoder() {
    codec_ = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec_) {
        throw std::runtime_error("H264 codec not found");
    }
    
    codecContext_ = avcodec_alloc_context3(codec_);
    if (!codecContext_) {
        throw std::runtime_error("Failed to allocate codec context");
    }
    
    // Set basic parameters
    codecContext_->width = current_width_;
    codecContext_->height = current_height_;
    codecContext_->time_base = {1, current_fps_};
    codecContext_->framerate = {current_fps_, 1};
    codecContext_->bit_rate = current_bitrate_ * 1000;
    codecContext_->gop_size = 10;
    codecContext_->max_b_frames = 0;  // No B-frames for low latency
    codecContext_->pix_fmt = AV_PIX_FMT_YUV420P;
    
    // Enhanced threading configuration
    codecContext_->thread_count = std::thread::hardware_concurrency();
    codecContext_->thread_type = FF_THREAD_FRAME;
    
    if (avcodec_open2(codecContext_, codec_, nullptr) < 0) {
        throw std::runtime_error("Failed to open codec");
    }
    
    // Allocate frame and packet
    frame_ = av_frame_alloc();
    pkt_ = av_packet_alloc();
    if (!frame_ || !pkt_) {
        throw std::runtime_error("Failed to allocate frame or packet");
    }
    
    frame_->format = codecContext_->pix_fmt;
    frame_->width = codecContext_->width;
    frame_->height = codecContext_->height;
    
    if (av_frame_get_buffer(frame_, 32) < 0) {
        throw std::runtime_error("Could not allocate frame buffer");
    }
    
    // Initialize swscale context
    swsCtx_ = sws_getContext(
        current_width_, current_height_, AV_PIX_FMT_BGR24,
        current_width_, current_height_, AV_PIX_FMT_YUV420P,
        SWS_BICUBIC, nullptr, nullptr, nullptr
    );
    
    if (!swsCtx_) {
        throw std::runtime_error("Failed to initialize swscale context");
    }
}

void FFmpegEncoder::configureLowLatency() {
    // Set optimal encoding parameters for low latency
    av_opt_set(codecContext_->priv_data, "threads", "auto", 0);
    av_opt_set(codecContext_->priv_data, "preset", "superfast", 0);
    av_opt_set(codecContext_->priv_data, "tune", "zerolatency", 0);
    av_opt_set(codecContext_->priv_data, "profile", "baseline", 0);
    
    // Low latency settings
    av_opt_set(codecContext_->priv_data, "bf", "0", 0);  // No B-frames
    av_opt_set(codecContext_->priv_data, "refs", "1", 0);  // Single reference frame
    av_opt_set(codecContext_->priv_data, "g", "10", 0);  // GOP size
    av_opt_set(codecContext_->priv_data, "keyint_min", "10", 0);
    av_opt_set(codecContext_->priv_data, "sc_threshold", "0", 0);  // Disable scene cut detection
    av_opt_set(codecContext_->priv_data, "flags", "+cgop", 0);  // Closed GOP
}

void FFmpegEncoder::updateEncoderParams() {
    // Update encoder parameters if needed
    if (codecContext_) {
        codecContext_->bit_rate = current_bitrate_ * 1000;
        codecContext_->time_base = {1, current_fps_};
        codecContext_->framerate = {current_fps_, 1};
    }
}

bool FFmpegEncoder::convertFrame(const cv::Mat& bgrFrame) {
    if (!frame_ || !swsCtx_) return false;
    
    // Prepare source data
    uint8_t* src_data[4] = {const_cast<uint8_t*>(bgrFrame.data), nullptr, nullptr, nullptr};
    int src_linesize[4] = {bgrFrame.step, 0, 0, 0};
    
    // Convert BGR to YUV420P
    int ret = sws_scale(swsCtx_, src_data, src_linesize, 0, current_height_,
                       frame_->data, frame_->linesize);
    
    if (ret < 0) {
        std::cerr << "[Encoder] Error converting frame format" << std::endl;
        return false;
    }
    
    frame_->pts = stats_.frames_encoded;
    return true;
}

void FFmpegEncoder::cleanup() {
    if (swsCtx_) {
        sws_freeContext(swsCtx_);
        swsCtx_ = nullptr;
    }
    
    if (pkt_) {
        av_packet_free(&pkt_);
    }
    
    if (frame_) {
        av_frame_free(&frame_);
    }
    
    if (codecContext_) {
        avcodec_free_context(&codecContext_);
    }
}

int FFmpegEncoder::calculateOptimalBitrate(double network_kbps, double loss_ratio) const {
    // Adaptive bitrate based on network conditions
    double target_bitrate = network_kbps * 0.8;  // Use 80% of available bandwidth
    
    // Reduce bitrate if loss is high
    if (loss_ratio > 0.05) {  // >5% loss
        target_bitrate *= (1.0 - loss_ratio);
    }
    
    // Apply bitrate tiers
    if (target_bitrate > 4000) {
        return 3000;  // 3 Mbps
    } else if (target_bitrate > 2000) {
        return 1800;  // 1.8 Mbps
    } else {
        return 1000;  // 1 Mbps
    }
}

int FFmpegEncoder::calculateOptimalFPS(double network_kbps) const {
    // Adaptive FPS based on network capacity
    if (network_kbps > 4000) {
        return 30;
    } else if (network_kbps > 2000) {
        return 25;
    } else {
        return 20;
    }
}

std::pair<int, int> FFmpegEncoder::calculateOptimalResolution(double network_kbps) const {
    // Adaptive resolution based on network capacity
    if (network_kbps > 4000) {
        return {1280, 720};  // 720p
    } else if (network_kbps > 2000) {
        return {960, 540};   // 540p
    } else {
        return {640, 360};   // 360p
    }
}

// Global interface functions
void initEncoder(int width, int height, int fps, int bitrate) {
    if (g_encoder) {
        delete g_encoder;
    }
    
    g_encoder = new FFmpegEncoder(width, height, fps, bitrate);
    std::cout << "[Encoder] ✅ Global encoder initialized" << std::endl;
}

bool encodeFrame(const cv::Mat& frame, std::vector<uint8_t>& data) {
    if (!g_encoder) {
        throw std::runtime_error("Encoder not initialized");
    }
    
    return g_encoder->encodeFrame(frame, data);
}

void setEncoderBitrate(int bitrate_kbps) {
    if (g_encoder) {
        g_encoder->setBitrate(bitrate_kbps);
    }
}

void adaptEncoderToNetwork(double network_kbps, double loss_ratio) {
    if (g_encoder) {
        g_encoder->adaptBitrate(network_kbps, loss_ratio);
    }
}

void shutdownEncoder() {
    if (g_encoder) {
        delete g_encoder;
        g_encoder = nullptr;
    }
    
    std::cout << "[Encoder] Encoder shutdown complete" << std::endl;
}
