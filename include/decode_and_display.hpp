#pragma once

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <opencv2/opencv.hpp>
#include <vector>

class H264Decoder {
public:
    H264Decoder();
    ~H264Decoder();
    
    bool init();
    bool decode(const std::vector<uint8_t>& encoded_data, cv::Mat& output_frame);
    void cleanup();
    
private:
    bool convertFrameToOpenCV(AVFrame* frame, cv::Mat& output_frame);
    
    AVCodec* codec;
    AVCodecContext* codec_ctx;
    AVPacket* packet;
    AVFrame* frame;
    struct SwsContext* sws_ctx;
    bool initialized;
};
