#include "decode_and_display.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

H264Decoder::H264Decoder() 
    : codec(nullptr), codec_ctx(nullptr), packet(nullptr), frame(nullptr), sws_ctx(nullptr), initialized(false) {
}

H264Decoder::~H264Decoder() {
    cleanup();
}

bool H264Decoder::init() {
    if (initialized) {
        return true;
    }
    
    codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) {
        std::cerr << "[DECODER] H264 decoder not found" << std::endl;
        return false;
    }
    
    codec_ctx = avcodec_alloc_context3(codec);
    if (!codec_ctx) {
        std::cerr << "[DECODER] Failed to allocate decoder context" << std::endl;
        return false;
    }
    
    if (avcodec_open2(codec_ctx, codec, nullptr) < 0) {
        std::cerr << "[DECODER] Failed to open decoder" << std::endl;
        return false;
    }
    
    packet = av_packet_alloc();
    frame = av_frame_alloc();
    if (!packet || !frame) {
        std::cerr << "[DECODER] Failed to allocate packet or frame" << std::endl;
        return false;
    }
    
    initialized = true;
    std::cout << "[DECODER] Initialized successfully" << std::endl;
    return true;
}

bool H264Decoder::decode(const std::vector<uint8_t>& encoded_data, cv::Mat& output_frame) {
    if (!initialized) {
        if (!init()) {
            return false;
        }
    }
    
    if (encoded_data.empty()) {
        std::cerr << "[DECODER] Empty input data" << std::endl;
        return false;
    }
    
    // Create AVPacket
    AVPacket packet;
    av_init_packet(&packet);
    packet.data = const_cast<uint8_t*>(encoded_data.data());
    packet.size = encoded_data.size();
    
    // Send packet to decoder
    int ret = avcodec_send_packet(codec_ctx, &packet);
    if (ret < 0) {
        std::cerr << "[DECODER] Error sending packet to decoder: " << ret << std::endl;
        return false;
    }
    
    // Receive frame from decoder
    AVFrame* frame = av_frame_alloc();
    if (!frame) {
        std::cerr << "[DECODER] Failed to allocate frame" << std::endl;
        return false;
    }
    
    ret = avcodec_receive_frame(codec_ctx, frame);
    if (ret < 0) {
        std::cerr << "[DECODER] Error receiving frame from decoder: " << ret << std::endl;
        av_frame_free(&frame);
        return false;
    }
    
    // Convert frame to OpenCV format
    if (!convertFrameToOpenCV(frame, output_frame)) {
        av_frame_free(&frame);
        return false;
    }
    
    av_frame_free(&frame);
    return true;
}

bool H264Decoder::convertFrameToOpenCV(AVFrame* frame, cv::Mat& output_frame) {
    if (!frame) {
        return false;
    }
    
    // Initialize sws context if needed
    if (!sws_ctx) {
        sws_ctx = sws_getContext(
            frame->width, frame->height, static_cast<AVPixelFormat>(frame->format),
            frame->width, frame->height, AV_PIX_FMT_BGR24,
            SWS_BICUBIC, nullptr, nullptr, nullptr
        );
    } else {
        // Update context if dimensions changed
        sws_ctx = sws_getCachedContext(
            sws_ctx,
            frame->width, frame->height, static_cast<AVPixelFormat>(frame->format),
            frame->width, frame->height, AV_PIX_FMT_BGR24,
            SWS_BICUBIC, nullptr, nullptr, nullptr
        );
    }
    
    if (!sws_ctx) {
        std::cerr << "[DECODER] Failed to create sws context" << std::endl;
        return false;
    }
    
    // Create output frame
    output_frame.create(frame->height, frame->width, CV_8UC3);
    uint8_t* dest[1] = { output_frame.data };
    int dest_stride[1] = { static_cast<int>(output_frame.step) };
    
    // Convert frame
    int scale_result = sws_scale(sws_ctx, frame->data, frame->linesize, 0, frame->height, dest, dest_stride);
    if (scale_result < 0) {
        std::cerr << "[DECODER] Failed to scale frame" << std::endl;
        return false;
    }
    
    std::cout << "[DECODER] Successfully decoded frame: " << frame->width << "x" << frame->height << std::endl;
    return true;
}

void H264Decoder::cleanup() {
    if (sws_ctx) {
        sws_freeContext(sws_ctx);
        sws_ctx = nullptr;
    }
    
    if (packet) {
        av_packet_free(&packet);
    }
    
    if (frame) {
        av_frame_free(&frame);
    }
    
    if (codec_ctx) {
        avcodec_free_context(&codec_ctx);
    }
    
    initialized = false;
}
