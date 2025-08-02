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
    stats_.frames_encoded++;
    stats_.total_bytes += outEncodedData.size();
    
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
    // Simple complexity calculation
    return 0.5;  // Fixed medium complexity
}

FFmpegEncoder::EncoderStats FFmpegEncoder::getStats() const {
    EncoderStats stats = stats_;
    stats.current_bitrate = current_bitrate_;
    stats.current_fps = current_fps_;
    stats.current_width = current_width_;
    stats.current_height = current_height_;
    
    return stats;
}

void FFmpegEncoder::adaptBitrate(double network_kbps, double loss_ratio, double rtt_ms) {
    // Calculate optimal parameters based on network conditions
    EncoderParams params = calculateOptimalParams(network_kbps, loss_ratio, rtt_ms);
    
    // Apply adaptations
    if (params.bitrate_kbps != current_bitrate_) {
        setBitrate(params.bitrate_kbps);
    }
    
    if (params.fps != current_fps_) {
        setFrameRate(params.fps);
    }
    
    // Apply advanced encoder parameters
    if (codecContext_ && codecContext_->priv_data) {
        // Set CRF (Constant Rate Factor) for quality control
        av_opt_set(codecContext_->priv_data, "crf", std::to_string(params.crf).c_str(), 0);
        
        // Set preset for speed/quality tradeoff
        av_opt_set(codecContext_->priv_data, "preset", params.preset.c_str(), 0);
        
        // Set keyframe interval
        av_opt_set(codecContext_->priv_data, "g", std::to_string(params.keyint).c_str(), 0);
        
        // Set reference frames
        av_opt_set(codecContext_->priv_data, "refs", std::to_string(params.refs).c_str(), 0);
        
        // Low latency mode
        if (params.low_latency) {
            av_opt_set(codecContext_->priv_data, "tune", "zerolatency", 0);
            av_opt_set(codecContext_->priv_data, "bf", "0", 0);  // No B-frames
        }
    }
    
    std::cout << "[Encoder] Adapted to network: " << network_kbps << " kbps, "
              << "loss: " << (loss_ratio * 100) << "%, RTT: " << rtt_ms << "ms, "
              << "new settings: " << params.bitrate_kbps << "kbps@" << params.fps 
              << "fps, CRF:" << params.crf << ", " << params.preset << std::endl;
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
    codecContext_->gop_size = 30;  // Increased GOP size for better quality
    codecContext_->max_b_frames = 0;  // No B-frames for low latency
    codecContext_->pix_fmt = AV_PIX_FMT_YUV420P;
    
    // Better threading
    codecContext_->thread_count = 2;  // Use 2 threads
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
    // Better quality settings
    av_opt_set(codecContext_->priv_data, "preset", "fast", 0);  // Better than ultrafast
    av_opt_set(codecContext_->priv_data, "tune", "zerolatency", 0);
    av_opt_set(codecContext_->priv_data, "profile", "baseline", 0);
    
    // Quality settings
    av_opt_set(codecContext_->priv_data, "bf", "0", 0);  // No B-frames
    av_opt_set(codecContext_->priv_data, "refs", "2", 0);  // 2 reference frames for better quality
    av_opt_set(codecContext_->priv_data, "g", "30", 0);  // GOP size
    av_opt_set(codecContext_->priv_data, "keyint_min", "30", 0);
    av_opt_set(codecContext_->priv_data, "sc_threshold", "40", 0);  // Scene cut detection
    av_opt_set(codecContext_->priv_data, "flags", "+cgop", 0);  // Closed GOP
    av_opt_set(codecContext_->priv_data, "me_method", "hex", 0);  // Motion estimation
    av_opt_set(codecContext_->priv_data, "subq", "6", 0);  // Subpixel quality
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
    int src_linesize[4] = {static_cast<int>(bgrFrame.step), 0, 0, 0};
    
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

int FFmpegEncoder::calculateOptimalBitrate(double network_kbps, double loss_ratio, double rtt_ms) const {
    // Advanced bitrate calculation formula
    // Optimal bitrate = network_capacity * efficiency_factor * quality_factor * latency_factor
    
    // Base efficiency (use 85% of available bandwidth)
    double efficiency_factor = 0.85;
    
    // Quality factor based on loss ratio
    double quality_factor = 1.0;
    if (loss_ratio > 0.01) {  // >1% loss
        quality_factor = 1.0 - (loss_ratio * 5.0);  // Reduce bitrate for high loss
    }
    
    // Latency factor based on RTT
    double latency_factor = 1.0;
    if (rtt_ms > 50.0) {  // >50ms RTT
        latency_factor = 1.0 - ((rtt_ms - 50.0) / 200.0);  // Reduce for high latency
    }
    
    // Calculate optimal bitrate
    double optimal_bitrate = network_kbps * efficiency_factor * quality_factor * latency_factor;
    
    // Apply bitrate tiers with quality preservation
    if (optimal_bitrate > 8000) {
        return 8000;  // 8 Mbps for excellent networks
    } else if (optimal_bitrate > 4000) {
        return 4000;  // 4 Mbps for good networks
    } else if (optimal_bitrate > 2000) {
        return 2500;  // 2.5 Mbps for medium networks
    } else if (optimal_bitrate > 1000) {
        return 1500;  // 1.5 Mbps for poor networks
    } else {
        return 1000;  // 1 Mbps minimum
    }
}

int FFmpegEncoder::calculateOptimalFPS(double network_kbps, double loss_ratio) const {
    // FPS calculation based on network capacity and loss
    if (network_kbps > 4000 && loss_ratio < 0.01) {
        return 30;  // 30fps for excellent conditions
    } else if (network_kbps > 2000 && loss_ratio < 0.02) {
        return 25;  // 25fps for good conditions
    } else if (network_kbps > 1000 && loss_ratio < 0.05) {
        return 20;  // 20fps for medium conditions
    } else {
        return 15;  // 15fps for poor conditions
    }
}

FFmpegEncoder::EncoderParams FFmpegEncoder::calculateOptimalParams(double network_kbps, double loss_ratio, double rtt_ms) const {
    EncoderParams params;
    
    // Calculate optimal bitrate and FPS
    params.bitrate_kbps = calculateOptimalBitrate(network_kbps, loss_ratio, rtt_ms);
    params.fps = calculateOptimalFPS(network_kbps, loss_ratio);
    
    // CRF (Constant Rate Factor) - Lower = better quality
    if (network_kbps > 4000 && loss_ratio < 0.01) {
        params.crf = 18;  // High quality
    } else if (network_kbps > 2000 && loss_ratio < 0.02) {
        params.crf = 20;  // Good quality
    } else if (network_kbps > 1000 && loss_ratio < 0.05) {
        params.crf = 23;  // Medium quality
    } else {
        params.crf = 26;  // Lower quality
    }
    
    // Preset selection based on network conditions
    if (rtt_ms < 50.0) {
        params.preset = "fast";  // Balance speed/quality for low latency
    } else if (rtt_ms < 100.0) {
        params.preset = "medium";  // Better quality for higher latency
    } else {
        params.preset = "slow";  // Best quality for high latency
    }
    
    // Keyframe interval based on FPS
    params.keyint = params.fps * 2;  // 2 seconds
    
    // Reference frames based on network conditions
    if (loss_ratio < 0.01) {
        params.refs = 3;  // More references for good networks
    } else if (loss_ratio < 0.02) {
        params.refs = 2;  // Medium references
    } else {
        params.refs = 1;  // Single reference for poor networks
    }
    
    // Low latency mode for real-time communication
    params.low_latency = (rtt_ms < 100.0);
    
    return params;
}

std::pair<int, int> FFmpegEncoder::calculateOptimalResolution(double network_kbps) const {
    // Always return 640x640 for consistency
    return {640, 640};
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

void adaptEncoderToNetwork(double network_kbps, double loss_ratio, double rtt_ms) {
    if (g_encoder) {
        g_encoder->adaptBitrate(network_kbps, loss_ratio, rtt_ms);
    }
}

void shutdownEncoder() {
    if (g_encoder) {
        delete g_encoder;
        g_encoder = nullptr;
    }
    
    std::cout << "[Encoder] Encoder shutdown complete" << std::endl;
}
