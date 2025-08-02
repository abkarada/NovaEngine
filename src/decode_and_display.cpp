#include "decode_and_display.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

H264Decoder::H264Decoder() {
    init();
}

void H264Decoder::init() {
    codec = const_cast<AVCodec*>(avcodec_find_decoder(AV_CODEC_ID_H264));
    if (!codec) throw std::runtime_error("H264 decoder not found");

    codec_ctx = avcodec_alloc_context3(codec);
    if (!codec_ctx) throw std::runtime_error("Failed to alloc codec context");

    codec_ctx->thread_count = 1;
    codec_ctx->thread_type = FF_THREAD_FRAME;

    if (avcodec_open2(codec_ctx, codec, nullptr) < 0)
        throw std::runtime_error("Could not open decoder");

    frame = av_frame_alloc();
    packet = av_packet_alloc();
}

bool H264Decoder::decode(const std::vector<uint8_t>& encoded_data, cv::Mat& output_frame) {
    if (!codec_ctx || !frame || !packet) {
        std::cerr << "[DECODER] Decoder not initialized!" << std::endl;
        return false;
    }

    std::cout << "[DECODER] Attempting to decode " << encoded_data.size() << " bytes" << std::endl;

    av_packet_unref(packet);
    packet->data = const_cast<uint8_t*>(encoded_data.data());
    packet->size = encoded_data.size();

    if (avcodec_send_packet(codec_ctx, packet) < 0) {
        std::cerr << "[DECODER] Failed to send packet to decoder" << std::endl;
        return false;
    }

    if (avcodec_receive_frame(codec_ctx, frame) == 0) {
        std::cout << "[DECODER] Successfully decoded frame: " 
                 << frame->width << "x" << frame->height << std::endl;
        
        sws_ctx = sws_getCachedContext(
            sws_ctx,
            frame->width, frame->height, static_cast<AVPixelFormat>(frame->format),
            frame->width, frame->height, AV_PIX_FMT_BGR24,
            SWS_BICUBIC, nullptr, nullptr, nullptr
        );

        output_frame.create(frame->height, frame->width, CV_8UC3);
        uint8_t* dest[1] = { output_frame.data };
        int dest_stride[1] = { static_cast<int>(output_frame.step) };

        sws_scale(sws_ctx, frame->data, frame->linesize, 0, frame->height, dest, dest_stride);
        std::cout << "[DECODER] Frame converted to OpenCV format: " 
                 << output_frame.cols << "x" << output_frame.rows << std::endl;
        return true;
    } else {
        std::cerr << "[DECODER] Failed to receive frame from decoder" << std::endl;
    }

    return false;
}

void H264Decoder::cleanup() {
    if (codec_ctx) avcodec_free_context(&codec_ctx);
    if (frame) av_frame_free(&frame);
    if (packet) av_packet_free(&packet);
    if (sws_ctx) sws_freeContext(sws_ctx);
}

H264Decoder::~H264Decoder() {
    cleanup();
}
