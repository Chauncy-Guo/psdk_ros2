#ifndef __PSDK_VEHICLE_WRAPPER_HH__
#define __PSDK_VEHICLE_WRAPPER_HH__

// Header include
#include "rclcpp/rclcpp.hpp"
#include <application.hpp>

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <sys/stat.h>
#include <cstring>
#ifdef ADVANCED_SENSING
#include <dji_camera_image.hpp>
#endif
#include <mutex>

//! psdk include
#include <utils/util_misc.h>
#include "utils/util_file.h"

#include "dji_logger.h"
#include "dji_platform.h"
#include "dji_typedef.h"
#include "dji_flight_controller.h"
#include "dji_fc_subscription.h"
#include "dji_gimbal_manager.h"
#include "dji_hms.h"
#include "dji_hms_info_table.h"
#include "dji_low_speed_data_channel.h"
#include "dji_high_speed_data_channel.h"
#include "dji_aircraft_info.h"
#include "dji_waypoint_v2.h"
#include "dji_waypoint_v3.h"
#include "dji_liveview.h"
#include "dji_camera_manager.h"
// #include "dji_mop_server.hpp"
#include "dji_mop_channel.h"

#define TEST_CAMERA_MOP_CHANNEL_SUBSCRIBE_POINT_CLOUD_CHANNEL_ID         49152
#define TEST_CAMERA_MOP_CHANNEL_SUBSCRIBE_POINT_CLOUD_RECV_BUFFER        (512 * 1024)
#define TEST_CAMERA_MOP_CHANNEL_COLOR_POINTS_BUFFER                      (512 * 1024)
#define TEST_CAMERA_MOP_CHANNEL_WAIT_TIME_MS                             (3 * 1000)
#define TEST_CAMERA_MOP_CHANNEL_MAX_RECV_COUNT                           30
#define TEST_CAMEAR_POINT_CLOUD_FILE_PATH_STR_MAX_SIZE                   256

static T_DjiMopChannelHandle s_mopChannelHandle;
static char s_pointCloudFilePath[TEST_CAMEAR_POINT_CLOUD_FILE_PATH_STR_MAX_SIZE];




const double C_EARTH = 6378137.0;
const double DEG2RAD = 0.01745329252;
#define TEST_CAMERA_MANAGER_MEDIA_FILE_NAME_MAX_SIZE             256


/* Private types -------------------------------------------------------------*/
#pragma pack(1)
typedef struct {
    dji_f32_t x;
    dji_f32_t y;
    dji_f32_t z;
} T_WindFlightControlVector3f; // pack(1)
#pragma pack()

typedef struct {
    E_DjiFcSubscriptionDisplayMode displayMode;
    char *displayModeStr;
} T_WindFlightControlDisplayModeStr;

/* Private values -------------------------------------------------------------*/
static const double s_earthCenter = 6378137.0;
static const double s_degToRad = 0.01745329252;

/* Hms private values */
static const char *oldReplaceAlarmIdStr = "%alarmid";
static const char *oldReplaceIndexStr = "%index";
static const char *oldReplaceComponentIndexStr = "%component_index";


static const T_WindFlightControlDisplayModeStr s_flightControlDisplayModeStr[] = {
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE, .displayModeStr = "attitude mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS, .displayModeStr = "p_gps mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF, .displayModeStr = "assisted takeoff mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF, .displayModeStr = "auto takeoff mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING, .displayModeStr = "auto landing mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME, .displayModeStr = "go home mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING, .displayModeStr = "force landing mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ENGINE_START, .displayModeStr = "engine start mode"}
    // {.displayMode = 0xFF, .displayModeStr = "unknown mode"}
};


/* Private types -------------------------------------------------------------*/
typedef struct {
    uint8_t eventID;
    char *eventStr;
} T_DjiTestWaypointV2EventStr;

typedef struct {
    uint8_t missionState;
    char *stateStr;
} T_DjiTestWaypointV2StateStr;

typedef struct {
    E_DjiCameraType cameraType;
    char  			*cameraTypeStr;
} T_DjiTestCameraTypeStr;

typedef struct {
	E_DjiCameraManagerWorkMode workMode;
	char *workModeStr;
} T_DjiCameraManagerWorkModeStr;

typedef struct {
	E_DjiCameraManagerShootPhotoMode shootPhotoMode;
	char *shootPhotoModeStr;
} T_DjiCameraManagerShootPhotoModeStr;

/* Private values -------------------------------------------------------------*/
//reference note of "T_DjiWaypointV2MissionEventPush"
static const T_DjiTestWaypointV2EventStr s_waypointV2EventStr[] = {
    {.eventID = 0x01, .eventStr = "Interrupt Event"},
    {.eventID = 0x02, .eventStr = "Resume Event"},
    {.eventID = 0x03, .eventStr = "Stop Event"},
    {.eventID = 0x10, .eventStr = "Arrival Event"},
    {.eventID = 0x11, .eventStr = "Finished Event"},
    {.eventID = 0x12, .eventStr = "Avoid Obstacle Event"},
    {.eventID = 0x30, .eventStr = "Action Switch Event"},
    {.eventID = 0xFF, .eventStr = "Unknown"}
};

//reference note of "T_DjiWaypointV2MissionStatePush"
static const T_DjiTestWaypointV2StateStr s_waypointV2StateStr[] = {
    {.missionState = 0x00, .stateStr = "Ground station not start"},
    {.missionState = 0x01, .stateStr = "Mission prepared"},
    {.missionState = 0x02, .stateStr = "Enter mission"},
    {.missionState = 0x03, .stateStr = "Execute mission"},
    {.missionState = 0x04, .stateStr = "Pause Mission"},
    {.missionState = 0x05, .stateStr = "Enter mission after ending pause"},
    {.missionState = 0x06, .stateStr = "Exit mission"},
    {.missionState = 0xFF, .stateStr = "Unknown"}
};

static const T_DjiTestCameraTypeStr s_cameraTypeStrList[] = {
    {DJI_CAMERA_TYPE_UNKNOWN, "Unknown"},
    {DJI_CAMERA_TYPE_Z30,     "Zenmuse Z30"},
    {DJI_CAMERA_TYPE_XT2,     "Zenmuse XT2"},
    {DJI_CAMERA_TYPE_PSDK,    "Payload Camera"},
    {DJI_CAMERA_TYPE_XTS,     "Zenmuse XTS"},
    {DJI_CAMERA_TYPE_H20,     "Zenmuse H20"},
    {DJI_CAMERA_TYPE_H20T,    "Zenmuse H20T"},
    {DJI_CAMERA_TYPE_P1,      "Zenmuse P1"},
    {DJI_CAMERA_TYPE_L1,      "Zenmuse L1"},
    {DJI_CAMERA_TYPE_L2,      "Zenmuse L2"},
    {DJI_CAMERA_TYPE_H20N,    "Zenmuse H20N"},
    {DJI_CAMERA_TYPE_M30,     "M30 Camera"},
    {DJI_CAMERA_TYPE_M30T,    "M30T Camera"},
    {DJI_CAMERA_TYPE_M3E,     "M3E Camera"},
    {DJI_CAMERA_TYPE_M3T,     "M3T Camera"},
    {DJI_CAMERA_TYPE_M3D,     "M3D Camera"},
    {DJI_CAMERA_TYPE_M3TD,    "M3TD Camera"},
    {DJI_CAMERA_TYPE_H30,     "H30 Camera"},
    {DJI_CAMERA_TYPE_H30T,    "H30T Camera"},
};


// Declaration
namespace dji_psdk_ros2
{
	class VehicleWrapper
	{
		public:
			VehicleWrapper(std::string config_json_path, std::string log_path);
			virtual ~VehicleWrapper() = default;

		public:
			/*! Parts of Camera */
			T_DjiReturnCode Camera_CameraManagerInit();
			T_DjiReturnCode Camera_CameraManagerDeInit();
			T_DjiReturnCode Camera_CameraManagerGetCameraType(E_DjiMountPosition position, E_DjiCameraType *cameraType);
			

		private:
			std::shared_ptr<Application> applicationObj_;
			
			// dji error publish function object
			std::function<void(uint64_t)> publishDjiErrorCode;
			std::string config_json_path_;
			std::string log_path_;
			bool hms_err_code_form_json_;
			
	};
}

#endif // __PSDK_VEHICLE_WRAPPER_HH__

