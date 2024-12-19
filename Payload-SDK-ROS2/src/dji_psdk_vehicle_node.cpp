
#include <dji_psdk_ros2/dji_psdk_vehicle_node.hpp>
#include "dji_psdk_vehicle_node.hpp"

using namespace cv;
using namespace dji_psdk_ros2;
using namespace std::placeholders;

VehicleWrapper* VehicleNode::ptr_wrapper_;
FFmpegStreamer* VehicleNode::streamer_;
FFmpegStreamer* VehicleNode::streamer_fpv_;

using namespace cv;
using namespace std;


rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr               VehicleNode::camera_h264_publisher_;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr               VehicleNode::camera_rgb_publisher_;
cv::VideoWriter VehicleNode::video_writer_;
bool            VehicleNode::video_writer_initialized_;

VehicleNode::VehicleNode()
{
    node_name_ = psdk_node->get_name();
    declareParameter();
    getParameter();
    initVehicleWrapper();
    osalHandler = DjiPlatform_GetOsalHandler();

    initCameraModule(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1);
    // initLiveViewModule();
    initLiveViewSample();
    initTopic();

    // Pub H264
    // osalHandler->TaskSleepMs(500);
    // T_DjiReturnCode returnCode;
    // returnCode = DjiLiveview_StartH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_NO_1, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT,publishCameraH264);
    // USER_LOG_INFO("StartH264Stream");

    // // show rgb image
    // char mainName[] = "MAIN_CAM";
    // liveviewSample_->StartMainCameraStream(&ShowRgbImageCallback, &mainName);


    // RTMP 配置
    int width = 640;
    int height = 480;
    int fps = 30;

    // 初始化 RTMP 推流
    // initRtmpStream(rtmp_url_, width, height, fps);
    streamer_     = new FFmpegStreamer(main_rtmp_url_, width, height, fps);
    streamer_fpv_ = new FFmpegStreamer(fpv_rtmp_url_, width, height, fps);

    // pub rgb image
    char mainName[] = "MAIN_CAM";
    liveviewSample_->StartMainCameraStream(&PublishRgbImage, &mainName);

    char fpvName[] = "FPV_CAM";
    liveviewSample_->StartFpvCameraStream(&PublishRgbImage_Fpv, &fpvName);

    Camera_CameraManagerSubscribePointCloudAndPub(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1);
}

VehicleNode::~VehicleNode(){
    delete ptr_wrapper_;
    delete liveviewSample_;
    ptr_wrapper_ = nullptr;
    USER_LOG_INFO("DJI PSDK VehicleNode Remove Success");
    rclcpp::shutdown();
};

void VehicleNode::declareParameter(){
    psdk_node->declare_parameter<std::string>("mainRtmpUrl", "rtmp://192.168.3.58:1935/live/233");
    psdk_node->declare_parameter<std::string>("fpvRtmpUrl", "rtmp://192.168.3.58:1935/live/266");
    psdk_node->declare_parameter<std::string>("configJsonPath", "~/ros_workspace/PSDK_ROS2/shells_ros2/Config/dji_sdk_config.json");
    psdk_node->declare_parameter<std::string>("LogFilePath",    "~/ros_workspace/Logs/");
    USER_LOG_INFO("declareParameter success");
}

void VehicleNode::getParameter(){
    psdk_node->get_parameter("mainRtmpUrl", main_rtmp_url_);
    psdk_node->get_parameter("fpvRtmpUrl",  fpv_rtmp_url_);
    psdk_node->get_parameter("configJsonPath", config_json_path_);
    psdk_node->get_parameter("LogFilePath",    log_file_path_);
    USER_LOG_INFO("getParameter success");
}

void VehicleNode::initVehicleWrapper(){
    ptr_wrapper_ = new(std::nothrow) VehicleWrapper(config_json_path_, log_file_path_);
    if(ptr_wrapper_ == nullptr){
        USER_LOG_ERROR("DJI PSDK Vehicle modules inited failed");
        rclcpp::shutdown();
    }
    USER_LOG_INFO("DJI PSDK VehicleNode Start");
}

void VehicleNode::getAircraftInfoBaseInfo(){
    T_DjiReturnCode returnCode;
    returnCode = DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get aircraft base info error");
    }
}

bool VehicleNode::initTopic()
{
    USER_LOG_INFO("Topic startup!");
    camera_h264_publisher_             = psdk_node->create_publisher<sensor_msgs::msg::Image>(node_name_ + "/camera_h264_stream", 10);
    camera_rgb_publisher_              = psdk_node->create_publisher<sensor_msgs::msg::Image>(node_name_ + "/main_camera_images", 1);
    return true;
}


bool VehicleNode::initCameraModule(E_DjiMountPosition position){
    T_DjiReturnCode returnCode;

    returnCode = DjiCameraManager_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Init camera manager failed, error code: 0x%08X\r\n", returnCode);
        return returnCode;
    }
    else
    {
        USER_LOG_INFO("Init camera manager successfully");
    }
    return true;
}

bool VehicleNode::initLiveViewModule(){
    T_DjiReturnCode returnCode;

    returnCode = DjiLiveview_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        perror("Liveview init failed");
    }

    return true;
}

bool VehicleNode::initLiveViewSample(){
    liveviewSample_ = new(std::nothrow) LiveviewSample();
    if(liveviewSample_ == nullptr){
        USER_LOG_ERROR("Failed to initialize liveviewSample_");
        return false;
    }
    return true;
}

void VehicleNode::publishCameraH264(E_DjiLiveViewCameraPosition position, const uint8_t *buf, uint32_t bufLen)
{
    std::vector<uint8_t> tempRawData(buf, buf+bufLen);
    sensor_msgs::msg::Image img;
    img.header.stamp = rclcpp::Clock().now();
    img.data = tempRawData;
    camera_h264_publisher_->publish(img);
}

void VehicleNode::ShowRgbImageCallback(CameraRGBImage img, void *userData)
{
    string name = string(reinterpret_cast<char *>(userData));
    Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width * 3);

    cvtColor(mat, mat, COLOR_RGB2BGR);
    imshow(name, mat);

    cv::waitKey(1);
}

void VehicleNode::PublishRgbImage(CameraRGBImage img, void *userData) {
    // 将 RGB 数据转换为 OpenCV Mat
    Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width * 3);
    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);

    // 转换为 ROS 2 图像消息并发布
    auto ros2ImageMsg = std::make_shared<sensor_msgs::msg::Image>();
    try {
        ros2ImageMsg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", mat).toImageMsg();
    } catch (cv_bridge::Exception &e) {
        return;
    }
    camera_rgb_publisher_->publish(*ros2ImageMsg);

    // 推流
    pushVideoStream(mat);
}

void VehicleNode::PublishRgbImage_Fpv(CameraRGBImage img, void *userData){
    Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width * 3);
    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
    streamer_fpv_->pushFrame(mat);
}

void VehicleNode::initRtmpStream(const std::string &rtmp_url, int width, int height, int fps) {
    // 初始化视频流
    if (video_writer_.isOpened()) {
        video_writer_.release();
    }

    int fourcc = VideoWriter::fourcc('f', 'm', 'p', '4'); // 使用 FFmpeg 的 MP4 格式
    video_writer_ = VideoWriter(rtmp_url, fourcc, fps, Size(width, height), true);

    if (!video_writer_.isOpened()) {
        RCLCPP_ERROR(psdk_node->get_logger(), "Failed to open RTMP stream: %s", rtmp_url.c_str());
    } else {
        RCLCPP_INFO(psdk_node->get_logger(), "RTMP stream initialized: %s", rtmp_url.c_str());
        video_writer_initialized_ = true;
    }
}

void VehicleNode::pushVideoStream(const cv::Mat &frame) {
    // // opencv + gstreamer的方式
    // if (video_writer_initialized_ && video_writer_.isOpened()) {
    //     video_writer_.write(frame); // 将视频帧写入 RTMP 流
    // }

    // ffmpeg的方式
    streamer_->pushFrame(frame);
}

T_DjiReturnCode VehicleNode::Camera_CameraManagerSubscribePointCloudAndPub(E_DjiMountPosition position) {
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    uint8_t recvBuf[TEST_CAMERA_MOP_CHANNEL_SUBSCRIBE_POINT_CLOUD_RECV_BUFFER] = {0};
    uint8_t colorPointsBuf[TEST_CAMERA_MOP_CHANNEL_COLOR_POINTS_BUFFER] = {0};
    uint32_t realLen;
    uint32_t recvDataCount = 0;
    struct tm *localTime = NULL;
    time_t currentTime = time(NULL);
    T_DjiCameraManagerColorPointCloud *colorPointCloud;
    uint32_t colorPointCloudDataByte = 0;
    uint32_t colorPointsNum = 0;
    static bool isMopInit = false;

    // Initialize ROS 2 Node and Publisher
    if (!psdk_ros2_pointcloud_node) {
        psdk_ros2_pointcloud_node = rclcpp::Node::make_shared("dji_point_cloud_node");
        point_cloud_publisher_ = psdk_ros2_pointcloud_node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
    }

    returnCode = DjiCameraManager_StartRecordPointCloud(position);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Start record point cloud failed, stat:0x%08llX.", returnCode);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    if (isMopInit == false) {
        returnCode = DjiMopChannel_Init();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Mop channel init error, stat:0x%08llX.", returnCode);
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        } else {
            isMopInit = true;
        }
    }

    returnCode = DjiMopChannel_Create(&s_mopChannelHandle, DJI_MOP_CHANNEL_TRANS_UNRELIABLE);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Mop channel create send handle error, stat:0x%08llX.", returnCode);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

RECONNECT:
    osalHandler->TaskSleepMs(TEST_CAMERA_MOP_CHANNEL_WAIT_TIME_MS);
    returnCode = DjiMopChannel_Connect(s_mopChannelHandle, DJI_CHANNEL_ADDRESS_PAYLOAD_PORT_NO1,
                                    TEST_CAMERA_MOP_CHANNEL_SUBSCRIBE_POINT_CLOUD_CHANNEL_ID);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Connect point cloud mop channel error, stat:0x%08llX.", returnCode);
        goto RECONNECT;
    }

    while (rclcpp::ok()) {
        memset(recvBuf, 0, TEST_CAMERA_MOP_CHANNEL_SUBSCRIBE_POINT_CLOUD_RECV_BUFFER);

        returnCode = DjiMopChannel_RecvData(s_mopChannelHandle, recvBuf,
                                            TEST_CAMERA_MOP_CHANNEL_SUBSCRIBE_POINT_CLOUD_RECV_BUFFER, &realLen);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            if (returnCode == DJI_ERROR_MOP_CHANNEL_MODULE_CODE_CONNECTION_CLOSE) {
                USER_LOG_INFO("Mop channel is disconnected");
                osalHandler->TaskSleepMs(TEST_CAMERA_MOP_CHANNEL_WAIT_TIME_MS);
                goto RECONNECT;
            }
        }

        colorPointCloud = (T_DjiCameraManagerColorPointCloud *) recvBuf;
        colorPointCloudDataByte = (colorPointCloud->pointCloudHeader).dataByte;
        colorPointsNum = colorPointCloudDataByte / sizeof(T_DjiCameraManagerPointXYZRGBInfo);

        // Prepare PointCloud2 message
        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud_msg->header.frame_id = "dji_point_cloud_frame";
        cloud_msg->header.stamp = psdk_ros2_pointcloud_node->get_clock()->now();
        cloud_msg->height = 1; // Unordered point cloud
        cloud_msg->width = colorPointsNum;
        cloud_msg->is_bigendian = false;
        cloud_msg->is_dense = true;
        cloud_msg->point_step = sizeof(T_DjiCameraManagerPointXYZRGBInfo);
        cloud_msg->row_step = cloud_msg->point_step * colorPointsNum;

        // Define fields
        sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

        // Copy data
        memcpy(colorPointsBuf, (uint8_t *)(colorPointCloud->points), colorPointCloudDataByte);
        cloud_msg->data.resize(colorPointCloudDataByte);
        memcpy(cloud_msg->data.data(), colorPointsBuf, colorPointCloudDataByte);

        // Publish the point cloud
        point_cloud_publisher_->publish(*cloud_msg);
        USER_LOG_INFO("Published point cloud data with %d points", colorPointsNum);
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}



int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    VehicleNode psdk_vh_node;

    rclcpp::spin(psdk_vh_node.psdk_node);
    
    return 0;
}