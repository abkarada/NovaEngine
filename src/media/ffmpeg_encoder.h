#pragma once

#include <vector>
#include <memory>
#include <mutex>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
}

class FFmpegEncoder {
public:
    FFmpegEncoder(int width, int height, int fps, int initial_bitrate_kbps);
    ~FFmpegEncoder();
    
    void reconfigure(int bitrate_kbps, int fps);
    std::vector<uint8_t> encode(const std::vector<uint8_t>& raw_frame_data);

private:
    void openCodec(int width, int height, int fps, int bitrate_kbps);
    void closeCodec();

    AVCodecContext* codec_ctx_;
    AVFrame* frame_;
    AVPacket* pkt_;
    int frame_width_;
    int frame_height_;
    int64_t pts;
    std::mutex codec_mutex_;
};
