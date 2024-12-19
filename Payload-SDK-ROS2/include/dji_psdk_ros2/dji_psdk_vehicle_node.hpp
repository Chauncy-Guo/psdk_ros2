#ifndef __DJI_PSDK_VEHICLE_NODE_HH__
#define __DJI_PSDK_VEHICLE_NODE_HH__

#include "rclcpp/rclcpp.hpp"
#include <ctime>
#include <sys/time.h>  // 使用 settimeofday
#include <chrono>

#include <dji_psdk_ros2/vehicle_wrapper.hpp>


#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "utils/util_misc.h"
#include "utils/cal_focus_enum.h"

#include "liveview/dji_camera_image_handler.hpp"
#include "liveview/dji_camera_stream_decoder.hpp"
#include "liveview/test_liveview_entry.hpp"
#include "liveview/test_liveview.hpp"

#include "ffmpeg_rtmp.hpp"

//! ROS standard msgs
#include "cv_bridge/cv_bridge.h"

#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/bool.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

namespace dji_psdk_ros2{
    class VehicleNode{
        public:
            VehicleNode();
            ~VehicleNode();

            void declareParameter();
            void getParameter();
            
            void initVehicleWrapper();
            void getAircraftInfoBaseInfo();

            bool initCameraModule(E_DjiMountPosition position);
            bool initLiveViewModule();
            bool initLiveViewSample();


            void initService();
            bool initTopic();
            void initSubscription();
            

        private:
            static void publishCameraH264(E_DjiLiveViewCameraPosition position, const uint8_t *buf, uint32_t bufLen);
            static void ShowRgbImageCallback(CameraRGBImage img, void *userData);
            static void PublishRgbImage(CameraRGBImage img, void *userData);
            static void PublishRgbImage_Fpv(CameraRGBImage img, void *userData);
            static void pushVideoStream(const cv::Mat &frame);

            void initRtmpStream(const std::string &rtmp_url, int width, int height, int fps);

        protected:
            bool checkCameraMode(int request_view, int request_source);
            
        public:
            T_DjiOsalHandler*       osalHandler;
            static VehicleWrapper*  ptr_wrapper_;
            LiveviewSample*         liveviewSample_;
            static FFmpegStreamer*  streamer_;
            static FFmpegStreamer*  streamer_fpv_;

            std::shared_ptr<rclcpp::Node> psdk_node   = rclcpp::Node::make_shared("dji_psdk_ros2");
            std::shared_ptr<rclcpp::Node> gimbal_node = rclcpp::Node::make_shared("psdk_ros_gimbal_node");
            rclcpp::Node::SharedPtr psdk_ros2_pointcloud_node = nullptr;

			T_DjiReturnCode Camera_CameraManagerSubscribePointCloudAndPub(E_DjiMountPosition position);


        private:
            T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;
            std::string node_name_;
            std::string config_json_path_;
            std::string log_file_path_;
          

			E_DjiCameraType cameraType_;
			uint8_t	        cameraTypeIndex_;
			T_DjiCameraManagerFirmwareVersion firmwareVersion_;

            std::string main_rtmp_url_;
            std::string fpv_rtmp_url_;
            static cv::VideoWriter video_writer_;
            static bool video_writer_initialized_;

        private:
            static rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr               camera_h264_publisher_;
            static rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr               camera_rgb_publisher_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr                point_cloud_publisher_ = nullptr;

    };
}



#endif // __DJI_PSDK_VEHICLE_NODE_HH__