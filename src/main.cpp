#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include <deque>
#include <atomic>

#include <camera/camera.h>
#include <camera/photography_settings.h>
#include <camera/device_discovery.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/imu.hpp"

struct TimeSync
{
  std::deque< std::pair<int64_t, int64_t> > pairs;
  double avg_delta = 0;
  int64_t update( int64_t sensor_ns, int64_t now_ns )
  {
    pairs.push_back({now_ns,sensor_ns});
    int k = 1;
    double a = 0.;
    for ( const auto & p : pairs )
    {
      int64_t d = p.first - p.second;
      a += (d - a)/k;
      ++k;
    }
    avg_delta = a;
    while(pairs.size() > 50){
      pairs.pop_front();
    }
    int64_t new_dt = sensor_ns + int64_t(avg_delta);
    return new_dt;
  }


  int64_t getTime(int64_t sensor_ns, bool a ) const
  {
    return sensor_ns + avg_delta;
  }
};

class TestStreamDelegate : public ins_camera::StreamDelegate {
private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    std::string imu_frame_ = "insta_imu";
    TimeSync gyro_sync;
    TimeSync cam_sync;

public:
    TestStreamDelegate(const std::shared_ptr<rclcpp::Node>& node) : node_(node) {
        // Publisher for the compressed H.264 video stream
        compressed_pub_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/insta/dual_fisheye/image/compressed",
            rclcpp::QoS(10)
        );

        // Publisher for IMU data (remains the same)
        imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/insta/imu/data_raw", 1000); // , rclcpp::SensorDataQoS(100));
        RCLCPP_INFO(node_->get_logger(), "Publisher for compressed images and IMU created.");
    }

    virtual ~TestStreamDelegate() {}

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override {}

    std::atomic<bool> first_time_ns_set = false; //already set?
    std::atomic<int64_t> first_time_ns; // measured by ros time now
    std::atomic<int64_t> first_time_ms; // measured by the camera

    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index) override {
        // We only care about the main video stream (index 0)
        if ( ! (stream_index == 0 && size > 0 && compressed_pub_)) return;

        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();

        const int64_t now_ns = node_->get_clock()->now().nanoseconds();
        const int64_t sensor_ns = int64_t(timestamp) * 1'000'000L; // ms to ns
        const int64_t scur_time_ns = cam_sync.update( sensor_ns, now_ns);
        int64_t cur_time_ns = std::min( scur_time_ns, now_ns );
        RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1.0, "CAM i: " << now_ns << " t: "<< sensor_ns << " t: " << cur_time_ns << " dt: " << ((cur_time_ns - now_ns)*1e-9) << " s: " << ((scur_time_ns - now_ns)*1e-9));

        rclcpp::Time t_dur (cur_time_ns, RCL_SYSTEM_TIME);

        // Set the header
        msg->header.stamp = t_dur;  //node_->get_clock()->now();
        msg->header.frame_id = "camera_frame";

        // Set the format to H.264
        // The subscriber will need to know this to select the correct decoder.
        msg->format = "h264";

        // Copy the compressed video data directly into the message
        msg->data.assign(data, data + size);
        compressed_pub_->publish(std::move(msg));
    }

    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override {
        if ( data.empty() ) return;
        static auto prev_time = node_->get_clock()->now();
        auto cur_time = node_->get_clock()->now();
        //static auto first_time = node_->get_clock()->now();
        //static int64_t first_time_ns  = (first_time.nanoseconds());

        //const int64_t cur_offset = data.empty() ? 0 : int64_t(data.back().timestamp); // first_time and back are closer in time...
        //static int64_t first_offset = data.empty() ? 0 : int64_t(data.back().timestamp); // first_time and back are closer in time...

        //if ( ! first_time_ns_set ) {
        //    first_time_ns = node_->get_clock()->now().nanoseconds();
        //    first_time_ms = data.empty() ? 0 : data.back().timestamp; // .back().timestamp is closer to now()
        //    first_time_ns_set = true;
        //}

        const int64_t now_ns = node_->get_clock()->now().nanoseconds();
        const int64_t sensor_ns = int64_t(data.back().timestamp) * 1'000'000L; // ms to ns
        const int64_t cur_time_ns = std::min(gyro_sync.update( sensor_ns, now_ns), now_ns);
        RCLCPP_INFO_STREAM(node_->get_logger(), "GYRO i: " << now_ns << " t: "<< sensor_ns << " t: " << cur_time_ns << " dt: " << ((cur_time_ns - now_ns)*1e-9));

        //static
        int nums = 0;
        for (const auto& gyro : data) {
            auto msg = std::make_unique<sensor_msgs::msg::Imu>();
            //msg->header.stamp = node_->get_clock()->now();
            int64_t time_since_first_stamp_ms = int64_t(gyro.timestamp) - int64_t(data.back().timestamp); // gyro.timestamp seems to be in ms!
            rclcpp::Time t_dur( cur_time_ns + time_since_first_stamp_ms * 1'000'000L, RCL_SYSTEM_TIME); // time since start...
            //msg->header.stamp = first_time + rclcpp::Duration(time_since_first_stamp_ms * 1'000'000L, RCL_SYSTEM_TIME); // TODO: correct this!  


            msg->header.stamp = t_dur;
            //const int64_t cur_time_ns = first_time_ns + time_since_first_stamp_ms*1'000'000L;
            //const int64_t ct_s = cur_time_ns / 1'000'000'000L;
            //const int64_t ct_ns = cur_time_ns - ct_s * 1'000'000'000L;
            //msg->header.stamp.sec = ct_s;
            //msg->header.stamp.nanosec = ct_ns;

            ++nums;
            //if ( nums < 100 )
            //RCLCPP_INFO_STREAM(node_->get_logger(), "i: " << nums << " t: "<< gyro.timestamp << " t: " << time_since_first_stamp_ms << " s: " << t_dur.nanoseconds() << " nano: " << first_time.nanoseconds() << " header: " << (msg->header.stamp.sec * 1'000'000'000L + msg->header.stamp.nanosec) << " ct: " << cur_time_ns);
            //if ( nums < 2)
            //RCLCPP_INFO_STREAM(node_->get_logger(), "i: " << nums << " t: "<< gyro.timestamp << " t: " << time_since_first_stamp_ms << " s: " << t_dur.nanoseconds() << " nano: " << first_time_ns << " header: " <<  " ct: " << cur_time_ns << " dt: " << (cur_time.nanoseconds()-cur_time_ns)*1e-9 );
            //RCLCPP_INFO_STREAM(node_->get_logger(), "i: " << nums << " t: "<< gyro.timestamp << " t: " << time_since_first_stamp_ms << " s: " << t_dur.nanoseconds() << " dt: " << ((now_ns - t_dur.nanoseconds())*1e-9) );

            msg->header.frame_id = imu_frame_;
            msg->angular_velocity.x = gyro.gx;
            msg->angular_velocity.y = gyro.gy;
            msg->angular_velocity.z = gyro.gz;
            
            constexpr double g_norm = 9.80665;
            msg->linear_acceleration.x = gyro.ax * g_norm;
            msg->linear_acceleration.y = gyro.ay * g_norm;
            msg->linear_acceleration.z = gyro.az * g_norm;

            msg->orientation.x = 0.0;
            msg->orientation.y = 0.0;
            msg->orientation.z = 0.0;
            msg->orientation.w = 1.0; // Neutral orientation
            msg->orientation_covariance[0] = -1.0; // No orientation data available

            for (int i = 0; i < 9; i++)
            {
                msg->angular_velocity_covariance[i] = 0;
                msg->linear_acceleration_covariance[i] = 0;
            }
            imu_pub_->publish(std::move(msg));
        }
        //RCLCPP_INFO_STREAM(node_->get_logger(), "got gyro: " << data.size() << " dt: " << (cur_time - prev_time).seconds());
        prev_time = cur_time;
    }

    void OnExposureData(const ins_camera::ExposureData& data) override {}
};

class CameraWrapper {
private:
    std::shared_ptr<ins_camera::Camera> cam;
    std::shared_ptr<rclcpp::Node> node_;

public:
    CameraWrapper(const std::shared_ptr<rclcpp::Node>& node) : node_(node) {}

    ~CameraWrapper() {
        if (cam) {
            cam->Close();
        }
    }

    int run_camera() {
        ins_camera::DeviceDiscovery discovery;
        auto list = discovery.GetAvailableDevices();
        if (list.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "No available camera devices found.");
            return -1;
        }

        const auto camera_type = list[0].camera_type;
        cam = std::make_shared<ins_camera::Camera>(list[0].info);
        if (!cam->Open()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open camera.");
            return -1;
        }
        RCLCPP_INFO(node_->get_logger(), "Camera opened successfully.");
        discovery.FreeDeviceDescriptors(list);


        // auto start = time(NULL);
        // localtime_r(&now, &tm);
        // time_seconds = timegm(&tm);

        // uint64_t utc_time = static_cast<uint64_t>(start);
        // uint32_t offset_time = 0; //no offset from UTC

        // //cam->SyncLocalTimeToCamera(utc_time,offset_time);       


        time_t now = time(nullptr);
        std::tm tm{};
        time_t time_seconds = now;
        localtime_r(&now, &tm);
        time_seconds = timegm(&tm);
    
        cam->SyncLocalTimeToCamera( now, time_seconds - now );
        
        
        RCLCPP_INFO_STREAM( node_->get_logger(), " UTC time: " << now << " GMT time: " << time_seconds << " cam media: " << cam->GetCameraMediaTime() );

        //const auto resolution = ins_camera::VideoResolution::RES_1440_720P30;
        //const auto resolution = ins_camera::VideoResolution::RES_1920_960P30;
        const auto resolution = ins_camera::VideoResolution::RES_3840_1920P30; // works reliably?
        //const auto resolution = ins_camera::VideoResolution::RES_2944_2880P30;

        // nope:
        // const auto resolution = ins_camera::VideoResolution::RES_3008_3008P24;
        // const auto resolution = ins_camera::VideoResolution::RES_3008_3008P25;
        // const auto resolution = ins_camera::VideoResolution::RES_3008_3008P30;
        // const auto resolution = ins_camera::VideoResolution::RES_3840_960P240;
        // const auto resolution = ins_camera::VideoResolution::RES_1920_1440P100;
        // const auto resolution = ins_camera::VideoResolution::RES_1920_1440P120;
        // const auto resolution = ins_camera::VideoResolution::RES_1440_1920P100;
        // const auto resolution = ins_camera::VideoResolution::RES_1440_1920P120;
        // const auto resolution = ins_camera::VideoResolution::RES_1920_1920P100;
        // const auto resolution = ins_camera::VideoResolution::RES_6KP30; 
        // const auto resolution = ins_camera::VideoResolution::RES_6KP25;
        // const auto resolution = ins_camera::VideoResolution::RES_6KP24;
        
        // disconnects:
        // const auto resolution = ins_camera::VideoResolution::RES_3840_960P200;
        // const auto resolution = ins_camera::VideoResolution::RES_8KP30;
        // const auto resolution = ins_camera::VideoResolution::RES_8KP25;
        // const auto resolution = ins_camera::VideoResolution::RES_8KP24;
        // const auto resolution = ins_camera::VideoResolution::RES_57K_PLUSP30;
        // const auto resolution = ins_camera::VideoResolution::RES_57K_PLUSP25;
        // const auto resolution = ins_camera::VideoResolution::RES_57K_PLUSP24;
        // const auto resolution = ins_camera::VideoResolution::RES_57KP60;
        // const auto resolution = ins_camera::VideoResolution::RES_57KP50;
        // const auto resolution = ins_camera::VideoResolution::RES_57KP48;
        // const auto resolution = ins_camera::VideoResolution::RES_57KP30;
        // const auto resolution = ins_camera::VideoResolution::RES_57KP25;
        // const auto resolution = ins_camera::VideoResolution::RES_57KP24:
        // const auto resolution = ins_camera::VideoResolution::RES_4KP100;
        // const auto resolution = ins_camera::VideoResolution::RES_4KP60;
        // const auto resolution = ins_camera::VideoResolution::RES_4KP50;
        // const auto resolution = ins_camera::VideoResolution::RES_4KP48;
        // const auto resolution = ins_camera::VideoResolution::RES_4KP30;
        // const auto resolution = ins_camera::VideoResolution::RES_4KP25;
        // const auto resolution = ins_camera::VideoResolution::RES_4KP24;
        // const auto resolution = ins_camera::VideoResolution::RES_3KP100;
        // const auto resolution = ins_camera::VideoResolution::RES_11KP30;
        // const auto resolution = ins_camera::VideoResolution::RES_57KP120;
        // const auto resolution = ins_camera::VideoResolution::RES_3KP240;


        // from sdk example:
        if (camera_type >= ins_camera::CameraType::Insta360X4) { // X4 or X5
            if (!cam->SetVideoSubMode(ins_camera::SubVideoMode::VIDEO_LIVEVIEW)) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to set mode.");
                return -1;
            }
            ins_camera::RecordParams record_params;
            record_params.resolution = resolution;
            record_params.bitrate = 0;
            if (!cam->SetVideoCaptureParams(record_params, ins_camera::CameraFunctionMode::FUNCTION_MODE_LIVE_STREAM)) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to set live streaming.");
                return -1;
            }
        }


        ins_camera::LiveStreamParam param;
        
        param.video_resolution = resolution;
        //Possible resolutions (results may vary per model) are:
        //RES_3840_1920P30
        //RES_2560_1280P30
        //RES_1152_1152P30 (this will give 2304 x 1152 at 30 FPS)
        //RES_1920_960P30  
        param.lrv_video_resulution = ins_camera::VideoResolution::RES_1440_720P30;
        //param.lrv_video_resulution = ins_camera::VideoResolution::RES_3840_1920P30;
        //param.video_bitrate = 1024 * 1024 / 2;
        param.enable_audio = false;
        param.using_lrv = false;

        std::shared_ptr<ins_camera::StreamDelegate> delegate = std::make_shared<TestStreamDelegate>(node_);
        cam->SetStreamDelegate(delegate);

        if ( ins_camera::VideoEncodeType::H264 == cam->GetVideoEncodeType() )
        {
            RCLCPP_INFO(node_->get_logger(), "Encoded to have H264!");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "should have H265!");
        }

        if (!cam->StartLiveStreaming(param)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to start live streaming.");
            return -1;
        }
        
        RCLCPP_INFO(node_->get_logger(), "Live streaming started.");
        return 0;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("insta_driver");
    
    CameraWrapper camera(node);
    if (camera.run_camera() != 0) {
        rclcpp::shutdown();
        return -1;
    }
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
