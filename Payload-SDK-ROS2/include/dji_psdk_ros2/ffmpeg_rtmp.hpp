extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

using namespace cv;

class FFmpegStreamer {
public:
    FFmpegStreamer(const std::string &rtmp_url, int width, int height, int fps)
        : width_(width), height_(height), fps_(fps), rtmp_url_(rtmp_url), initialized_(false) {
        initializeFFmpeg();
    }

    ~FFmpegStreamer() {
        if (initialized_) {
            av_write_trailer(output_ctx_);
            avcodec_free_context(&codec_ctx_);
            avformat_free_context(output_ctx_);
            sws_freeContext(sws_ctx_);
        }
    }
    
    void pushFrame(const cv::Mat &frame) {
        if (!initialized_) {
            std::cerr << "FFmpeg not initialized properly. Cannot push frame." << std::endl;
            return;
        }

        // Convert OpenCV Mat to YUV format
        AVFrame *yuv_frame = av_frame_alloc();
        yuv_frame->format = codec_ctx_->pix_fmt;
        yuv_frame->width = codec_ctx_->width;
        yuv_frame->height = codec_ctx_->height;

        if (av_frame_get_buffer(yuv_frame, 32) < 0) {
            std::cerr << "Failed to allocate buffer for YUV frame." << std::endl;
            av_frame_free(&yuv_frame);
            return;
        }

        // Convert frame to YUV
        cv::Mat resized_frame;
        cv::resize(frame, resized_frame, cv::Size(width_, height_));

        const uint8_t *src_data[1] = { resized_frame.data };
        int src_linesize[1] = { static_cast<int>(resized_frame.step) };

        sws_scale(sws_ctx_, src_data, src_linesize, 0, height_,
                yuv_frame->data, yuv_frame->linesize);

        // Encode and send frame
        sendFrame(yuv_frame);

        av_frame_free(&yuv_frame);
    }

private:
    std::string rtmp_url_;
    int width_, height_, fps_;
    AVFormatContext *output_ctx_ = nullptr;
    AVCodecContext *codec_ctx_ = nullptr;
    AVStream *video_stream_ = nullptr;
    SwsContext *sws_ctx_ = nullptr;
    int64_t pts_ = 0;
    bool initialized_ = false;

    void initializeFFmpeg() {
        avformat_network_init();

        // Create output context
        if (avformat_alloc_output_context2(&output_ctx_, nullptr, "flv", rtmp_url_.c_str()) < 0) {
            std::cerr << "Could not allocate output context." << std::endl;
            return;
        }

        // Find encoder
        const AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H264);
        if (!codec) {
            std::cerr << "H.264 encoder not found." << std::endl;
            return;
        }

        // Create codec context
        codec_ctx_ = avcodec_alloc_context3(codec);
        codec_ctx_->width = width_;
        codec_ctx_->height = height_;
        codec_ctx_->time_base = {1, fps_};
        codec_ctx_->framerate = {fps_, 1};
        codec_ctx_->gop_size = 10;
        codec_ctx_->max_b_frames = 1;
        codec_ctx_->pix_fmt = AV_PIX_FMT_YUV420P;

        if (output_ctx_->oformat->flags & AVFMT_GLOBALHEADER) {
            codec_ctx_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
        }

        if (avcodec_open2(codec_ctx_, codec, nullptr) < 0) {
            std::cerr << "Could not open codec." << std::endl;
            return;
        }

        // Create video stream
        video_stream_ = avformat_new_stream(output_ctx_, nullptr);
        if (!video_stream_) {
            std::cerr << "Could not create video stream." << std::endl;
            return;
        }

        video_stream_->time_base = {1, fps_};
        video_stream_->codecpar->codec_tag = 0;
        if (avcodec_parameters_from_context(video_stream_->codecpar, codec_ctx_) < 0) {
            std::cerr << "Could not copy codec parameters." << std::endl;
            return;
        }

        // Open output file
        if (avio_open(&output_ctx_->pb, rtmp_url_.c_str(), AVIO_FLAG_WRITE) < 0) {
            std::cerr << "Could not open output file." << std::endl;
            return;
        }

        // Write header
        if (avformat_write_header(output_ctx_, nullptr) < 0) {
            std::cerr << "Could not write header." << std::endl;
            return;
        }

        // Create scaling context
        sws_ctx_ = sws_getContext(width_, height_, AV_PIX_FMT_BGR24,
                                  width_, height_, AV_PIX_FMT_YUV420P,
                                  SWS_BICUBIC, nullptr, nullptr, nullptr);
        if (!sws_ctx_) {
            std::cerr << "Could not create scaling context." << std::endl;
            return;
        }

        initialized_ = true;
    }

    void sendFrame(AVFrame *frame) {
        frame->pts = pts_++;

        // Encode frame
        AVPacket pkt;
        av_init_packet(&pkt);
        pkt.data = nullptr;
        pkt.size = 0;

        if (avcodec_send_frame(codec_ctx_, frame) < 0) {
            std::cerr << "Failed to send frame for encoding." << std::endl;
            return;
        }

        while (avcodec_receive_packet(codec_ctx_, &pkt) == 0) {
            pkt.stream_index = video_stream_->index;
            pkt.pts = av_rescale_q(pkt.pts, codec_ctx_->time_base, video_stream_->time_base);
            pkt.dts = av_rescale_q(pkt.dts, codec_ctx_->time_base, video_stream_->time_base);
            pkt.duration = av_rescale_q(pkt.duration, codec_ctx_->time_base, video_stream_->time_base);

            if (av_interleaved_write_frame(output_ctx_, &pkt) < 0) {
                std::cerr << "Failed to write packet to output." << std::endl;
            }

            av_packet_unref(&pkt);
        }
    }
};