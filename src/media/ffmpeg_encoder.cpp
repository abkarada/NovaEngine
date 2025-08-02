// ffmpeg_encoder.cpp
#include "ffmpeg_encoder.h"
#include <stdexcept>
#include <iostream>

FFmpegEncoder::FFmpegEncoder(int width, int height, int fps, int initial_bitrate_kbps)
    : frame_width_(width), frame_height_(height) {
    
    frame_ = av_frame_alloc();
    pkt_ = av_packet_alloc();
    if (!frame_ || !pkt_) {
        throw std::runtime_error("Could not allocate AVFrame or AVPacket");
    }

    frame_->width = frame_width_;
    frame_->height = frame_height_;
    frame_->format = AV_PIX_FMT_YUV420P; // x264 requires YUV

    av_frame_get_buffer(frame_, 0);
    
    openCodec(width, height, fps, initial_bitrate_kbps);
}

FFmpegEncoder::~FFmpegEncoder() {
    closeCodec();
    av_frame_free(&frame_);
    av_packet_free(&pkt_);
}

void FFmpegEncoder::openCodec(int width, int height, int fps, int bitrate_kbps) {
    const AVCodec* codec = avcodec_find_encoder_by_name("libx264");
    if (!codec) {
        throw std::runtime_error("Codec libx264 not found");
    }
    
    codec_ctx_ = avcodec_alloc_context3(codec);
    if (!codec_ctx_) {
        throw std::runtime_error("Could not allocate video codec context");
    }

    codec_ctx_->bit_rate = bitrate_kbps * 1000;
    codec_ctx_->width = width;
    codec_ctx_->height = height;
    codec_ctx_->time_base = {1, fps};
    codec_ctx_->framerate = {fps, 1};
    codec_ctx_->gop_size = fps; // Keyframe every second
    codec_ctx_->max_b_frames = 0; // No B-frames for low latency
    codec_ctx_->pix_fmt = AV_PIX_FMT_YUV420P;

    // Netflix-quality and low-latency settings
    av_opt_set(codec_ctx_->priv_data, "preset", "veryfast", 0);
    av_opt_set(codec_ctx_->priv_data, "tune", "film", 0);
    av_opt_set(codec_ctx_->priv_data, "threads", "auto", 0); // Multi-threading

    if (avcodec_open2(codec_ctx_, codec, nullptr) < 0) {
        throw std::runtime_error("Could not open codec");
    }
    std::cout << "Opened encoder: " << bitrate_kbps << " kbps @" << fps << "fps" << std::endl;
}

void FFmpegEncoder::closeCodec() {
    if (codec_ctx_) {
        avcodec_free_context(&codec_ctx_);
        codec_ctx_ = nullptr;
    }
}

void FFmpegEncoder::reconfigure(int bitrate_kbps, int fps) {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    // A simple approach is to close and reopen the codec.
    // Some parameters can be changed on the fly, but bitrate/fps often require a full reinit.
    closeCodec();
    openCodec(frame_width_, frame_height_, fps, bitrate_kbps);
}

std::vector<uint8_t> FFmpegEncoder::encode(const std::vector<uint8_t>& raw_frame_data) {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    // This is a placeholder for color conversion (e.g., BGR -> YUV420p)
    // You would use libswscale for this in a real app.
    // For now, we assume the input is already YUV420p.
    // A simplified copy for demonstration:
    // This part is CRITICAL and needs a real sws_scale implementation.
    int in_linesize[1] = { frame_width_ * 3 }; // Assuming BGR input
    // sws_scale(...);
    // For this example, let's just pretend frame_ is filled.

    static int64_t pts = 0;
    frame_->pts = pts++;

    int ret = avcodec_send_frame(codec_ctx_, frame_);
    if (ret < 0) {
        return {}; // Error
    }

    ret = avcodec_receive_packet(codec_ctx_, pkt_);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        return {}; // Need more frames or end of stream
    } else if (ret < 0) {
        return {}; // Error
    }

    std::vector<uint8_t> encoded_data(pkt_->data, pkt_->data + pkt_->size);
    av_packet_unref(pkt_);
    
    return encoded_data;
}