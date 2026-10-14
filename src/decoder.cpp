#include <iostream>
#include <thread>
#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libswscale/swscale.h>
    #include <libavutil/imgutils.h>
}

static enum AVPixelFormat get_hw_format(AVCodecContext *ctx, const enum AVPixelFormat *pix_fmts) {
    const enum AVPixelFormat *p;
    for (p = pix_fmts; *p != AV_PIX_FMT_NONE; p++) {
        if (*p == AV_PIX_FMT_CUDA) {
            return *p;
        }
    }
    return AV_PIX_FMT_NONE;
}

class H264DecoderNode : public rclcpp::Node {
private:
    const AVCodec* codec_ = nullptr;
    AVCodecContext* codec_ctx_ = nullptr;
    AVCodecParserContext* parser_ctx_ = nullptr;
    AVPacket* pkt_ = nullptr;
    AVFrame* hw_frame_ = nullptr;
    AVFrame* sw_frame_ = nullptr;
    SwsContext* sws_ctx_ = nullptr;
    cv::Mat bgr_frame_;
    AVBufferRef *hw_device_ctx_ = nullptr;
    enum AVHWDeviceType hw_type_ = AV_HWDEVICE_TYPE_NONE;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    image_transport::CameraPublisher publisher_cam0_;
    image_transport::CameraPublisher publisher_cam1_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_cam0_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_cam1_;


    std::thread publisher_thread_;
    std::queue<std::pair<cv::Mat,int64_t>> frame_publish_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::atomic<bool> stop_publisher_thread_{false};
    size_t max_queue_size_ = 10;
    
    int skip_frame_ = 0;
    int frame_counter_ = 0;
    bool i_frame_only_ = false;
    
    bool rotate_inwards_ = false;
    bool rotate_outwards_ = false;

    void InitFFmpegDecoder() {
        hw_type_ = AV_HWDEVICE_TYPE_CUDA;
        const char* decoder_name = "h264_cuvid";

        codec_ = avcodec_find_decoder_by_name(decoder_name);
        if (!codec_) {
            RCLCPP_WARN(this->get_logger(), "Hardware decoder not available, falling back to software");
            hw_type_ = AV_HWDEVICE_TYPE_NONE;
            codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
            if (!codec_) {
                RCLCPP_ERROR(this->get_logger(), "No H.264 decoder available");
                return;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Using hardware H.264 decoder (NVDEC)");
        }

        if (hw_type_ != AV_HWDEVICE_TYPE_NONE) {
            int err = av_hwdevice_ctx_create(&hw_device_ctx_, hw_type_, nullptr, nullptr, 0);
            if (err < 0) {
                RCLCPP_WARN(this->get_logger(), "Failed to create hardware device context, falling back to software");
                hw_type_ = AV_HWDEVICE_TYPE_NONE;
                codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
                if (!codec_) {
                    RCLCPP_ERROR(this->get_logger(), "No H.264 decoder available");
                    return;
                }
            }
        }

        parser_ctx_ = av_parser_init(codec_->id);
        if (!parser_ctx_) {
            CleanupFFmpegDecoder();
            return;
        }

        codec_ctx_ = avcodec_alloc_context3(codec_);
        if (!codec_ctx_) {
            CleanupFFmpegDecoder();
            return;
        }

        if (hw_type_ != AV_HWDEVICE_TYPE_NONE && hw_device_ctx_) {
            codec_ctx_->hw_device_ctx = av_buffer_ref(hw_device_ctx_);
            codec_ctx_->get_format = get_hw_format;
        }

        if (avcodec_open2(codec_ctx_, codec_, nullptr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open codec");
            CleanupFFmpegDecoder();
            return;
        }

        pkt_ = av_packet_alloc();
        if (!pkt_) {
            CleanupFFmpegDecoder();
            return;
        }

        hw_frame_ = av_frame_alloc();
        if (!hw_frame_) {
            CleanupFFmpegDecoder();
            return;
        }

        if (hw_type_ != AV_HWDEVICE_TYPE_NONE) {
            sw_frame_ = av_frame_alloc();
            if (!sw_frame_) {
                CleanupFFmpegDecoder();
                return;
            }
        }
    }

    void PublisherThreadLoop() {
        while (!stop_publisher_thread_) {
            cv::Mat frame_to_publish;
            int64_t frame_time_publish = 0;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [this] {
                    return !frame_publish_queue_.empty() || stop_publisher_thread_;
                });

                if (stop_publisher_thread_ && frame_publish_queue_.empty()) {
                    break;
                }
                if (frame_publish_queue_.empty()) {
                    continue;
                }
                frame_to_publish = frame_publish_queue_.front().first;
                frame_time_publish = frame_publish_queue_.front().second;
                frame_publish_queue_.pop();
            }

            if (!frame_to_publish.empty() && publisher_) {


                auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
                std_msgs::msg::Header header;
                header.stamp = rclcpp::Time(frame_time_publish, RCL_SYSTEM_TIME); //this->get_clock()->now();
                header.frame_id = "camera_optical_frame";
                cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8, frame_to_publish);

                if ( publisher_cam0_.getNumSubscribers() > 0 || publisher_cam1_.getNumSubscribers() > 0 )
                {

                    const int half_width = frame_to_publish.cols / 2;
                    const int height = frame_to_publish.rows;
                    cv::Mat left_mat = frame_to_publish(cv::Rect(0, 0, half_width, height));
                    cv::Mat right_mat = frame_to_publish(cv::Rect(half_width, 0, half_width, height));

                    if ( rotate_inwards_ )
                    {
                        cv::rotate( left_mat, left_mat, cv::ROTATE_90_CLOCKWISE);
                        cv::rotate( right_mat, right_mat, cv::ROTATE_90_COUNTERCLOCKWISE);
                    }
                    else if ( rotate_outwards_ )
                    {
                        cv::rotate( left_mat, left_mat, cv::ROTATE_90_COUNTERCLOCKWISE);
                        cv::rotate( right_mat, right_mat, cv::ROTATE_90_CLOCKWISE);
                    }

                    sensor_msgs::msg::Image::SharedPtr left_img_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, left_mat).toImageMsg();
                    sensor_msgs::msg::Image::SharedPtr right_img_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, right_mat).toImageMsg();

                    sensor_msgs::msg::CameraInfo left_info = cinfo_manager_cam0_->getCameraInfo();
                    left_info.header.stamp = header.stamp;
                    left_info.header.frame_id = "cam0_optical_frame";
                    left_info.width = half_width;
                    left_info.height = height;

                    sensor_msgs::msg::CameraInfo right_info = cinfo_manager_cam1_->getCameraInfo();
                    right_info.header.stamp = header.stamp;
                    right_info.header.frame_id = "cam1_optical_frame";
                    right_info.width = half_width;
                    right_info.height = height;

                    publisher_cam0_.publish(*left_img_msg, left_info);
                    publisher_cam1_.publish(*right_img_msg, right_info);
                }
                if ( publisher_->get_subscription_count() > 0 )
                {
                    cv_image.toImageMsg(*img_msg);
                    publisher_->publish(std::move(img_msg));
                }
            }
        }
    }

    void DecodeAndDisplayPacket(AVPacket* packet, const int64_t & timestamp_ns ) {
        int ret = avcodec_send_packet(codec_ctx_, packet);
        if (ret < 0) {
            return;
        }

        while (ret >= 0) {
            ret = avcodec_receive_frame(codec_ctx_, hw_frame_);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                return;
            } else if (ret < 0) {
                return;
            }

            AVFrame* frame_to_display = hw_frame_;

            if (hw_frame_->format == AV_PIX_FMT_CUDA || hw_frame_->format == AV_PIX_FMT_VAAPI) {
                if (av_hwframe_transfer_data(sw_frame_, hw_frame_, 0) < 0) {
                    av_frame_unref(hw_frame_);
                    continue;
                }
                frame_to_display = sw_frame_;
            }

            if (!sws_ctx_ && frame_to_display->width > 0 && frame_to_display->height > 0) {
                sws_ctx_ = sws_getContext(
                    frame_to_display->width, frame_to_display->height, (AVPixelFormat)frame_to_display->format,
                    frame_to_display->width, frame_to_display->height, AV_PIX_FMT_BGR24,
                    SWS_POINT, nullptr, nullptr, nullptr);
                
                if (!sws_ctx_) {
                    av_frame_unref(hw_frame_);
                    if (frame_to_display == sw_frame_) av_frame_unref(sw_frame_);
                    return;
                }
                bgr_frame_.create(frame_to_display->height, frame_to_display->width, CV_8UC3);
            }

            if (sws_ctx_ && !bgr_frame_.empty()) {
                uint8_t* dst_data[4] = { bgr_frame_.data, nullptr, nullptr, nullptr };
                int dst_linesize[4] = { static_cast<int>(bgr_frame_.step[0]), 0, 0, 0 };

                sws_scale(sws_ctx_,
                            (const uint8_t* const*)frame_to_display->data, frame_to_display->linesize,
                            0, frame_to_display->height,
                            dst_data, dst_linesize);

                // Apply frame skipping after decoding
                bool should_publish = true;
                
                if (skip_frame_ > 0 && !i_frame_only_) {
                    // Skip frame logic (only when not in i_frame_only mode)
                    should_publish = (frame_counter_++ % (skip_frame_ + 1) == 0);
                }
                
                if (should_publish) {
                    cv::Mat frame_copy = bgr_frame_.clone();
                    {
                        std::lock_guard<std::mutex> lock(queue_mutex_);
                        if (frame_publish_queue_.size() < max_queue_size_) {
                            frame_publish_queue_.push({frame_copy,timestamp_ns});
                        }
                    }
                    queue_cv_.notify_one();
                }
            }
            
            av_frame_unref(hw_frame_);
            if (frame_to_display == sw_frame_) {
                av_frame_unref(sw_frame_);
            }
        }
    }

    void CleanupFFmpegDecoder() {
        if (sws_ctx_) {
            sws_freeContext(sws_ctx_);
            sws_ctx_ = nullptr;
        }
        if (sw_frame_) {
            av_frame_free(&sw_frame_);
            sw_frame_ = nullptr;
        }
        if (hw_frame_) {
            av_frame_free(&hw_frame_);
            hw_frame_ = nullptr;
        }
        if (pkt_) {
            av_packet_free(&pkt_);
            pkt_ = nullptr;
        }
        if (codec_ctx_) {
            avcodec_close(codec_ctx_);
            avcodec_free_context(&codec_ctx_);
            codec_ctx_ = nullptr;
        }
        if (parser_ctx_) {
            av_parser_close(parser_ctx_);
            parser_ctx_ = nullptr;
        }
        if (hw_device_ctx_) {
            av_buffer_unref(&hw_device_ctx_);
            hw_device_ctx_ = nullptr;
        }
        codec_ = nullptr;
    }

    void compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        if (msg->format != "h264") {
            return;
        }

        if (!codec_ctx_ || !parser_ctx_ || !pkt_ || !hw_frame_) {
            return;
        }

        const uint8_t* cur_data = msg->data.data();
        size_t remaining_size = msg->data.size();

        while (remaining_size > 0) {
            int bytes_parsed = av_parser_parse2(parser_ctx_, codec_ctx_,
                                                &pkt_->data, &pkt_->size,
                                                cur_data, static_cast<int>(remaining_size),
                                                AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
            if (bytes_parsed < 0) {
                break;
            }
            cur_data += bytes_parsed;
            remaining_size -= bytes_parsed;

            if (pkt_->size <= 0) continue;

            // Check if this is an I-frame when i_frame_only mode is enabled
            if (i_frame_only_ && parser_ctx_->key_frame != 1) continue;

            // Parse NAL unit type from H.264 stream
            // The parser sets keyframe flag for I-frames
            DecodeAndDisplayPacket(pkt_, rclcpp::Time(msg->header.stamp).nanoseconds());
        }
    }

public:
    H264DecoderNode() : Node("h264_decoder_node") {
        this->declare_parameter("compressed_topic", "/insta/dual_fisheye/image/compressed");
        this->declare_parameter("uncompressed_topic", "/insta/dual_fisheye/image");
        this->declare_parameter("skip_frame", 0);
        this->declare_parameter("i_frame_only", false);
        this->declare_parameter("rotate_inwards", false);
        this->declare_parameter("rotate_outwards", false);

        std::string subscribe_topic = this->get_parameter("compressed_topic").as_string();
        std::string publish_topic = this->get_parameter("uncompressed_topic").as_string();
        skip_frame_ = this->get_parameter("skip_frame").as_int();
        i_frame_only_ = this->get_parameter("i_frame_only").as_bool();
        rotate_inwards_ = this->get_parameter("rotate_inwards").as_bool();
        rotate_outwards_ = this->get_parameter("rotate_outwards").as_bool();

        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            subscribe_topic, 10,
            std::bind(&H264DecoderNode::compressed_image_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(publish_topic, 10);

        std::string cinfo_url_0 = "package://insta360_ros_driver/config/cam0.yaml";
        std::string cinfo_url_1 = "package://insta360_ros_driver/config/cam1.yaml";
        cinfo_manager_cam0_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "cam0", cinfo_url_0);
        cinfo_manager_cam1_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "cam1", cinfo_url_1);
        publisher_cam0_ = image_transport::create_camera_publisher(this, "insta/cam0/image_raw");
        publisher_cam1_ = image_transport::create_camera_publisher(this, "insta/cam1/image_raw");

        publisher_thread_ = std::thread(&H264DecoderNode::PublisherThreadLoop, this);
        
        InitFFmpegDecoder();

        RCLCPP_INFO(this->get_logger(), "H.264 Decoder Node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", subscribe_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", publish_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Skip frame: %d, I-frame only: %s", skip_frame_, i_frame_only_ ? "true" : "false");
    }

    ~H264DecoderNode() {
        stop_publisher_thread_ = true;
        queue_cv_.notify_one();
        if (publisher_thread_.joinable()) {
            publisher_thread_.join();
        }
        CleanupFFmpegDecoder();
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<H264DecoderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
