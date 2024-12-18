

#include <vehicle_wrapper.hpp>
namespace dji_psdk_ros2{
    /*! Parts of Camera */
    T_DjiReturnCode VehicleWrapper::Camera_CameraManagerInit()
    {
        T_DjiReturnCode returnCode;

        returnCode = DjiCameraManager_Init();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            publishDjiErrorCode(returnCode);
            USER_LOG_ERROR("Init camera manager failed, error code: 0x%08X\r\n", returnCode);
            return returnCode;
        }
        else
        {
            USER_LOG_INFO("Init camera manager successfully");
        }

        return returnCode;
    }

    T_DjiReturnCode VehicleWrapper::Camera_CameraManagerDeInit()
    {
        T_DjiReturnCode returnCode;

        returnCode = DjiCameraManager_DeInit();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            publishDjiErrorCode(returnCode);
            USER_LOG_ERROR("DeInit camera manager failed, error code: 0x%08X\r\n", returnCode);
            return returnCode;
        }
        else
        {
            USER_LOG_INFO("DeInit camera manager successfully");
        }

        return returnCode;
    }

    T_DjiReturnCode VehicleWrapper::Camera_CameraManagerGetCameraType(E_DjiMountPosition position, E_DjiCameraType *cameraType)
    {
        T_DjiReturnCode returnCode;

        returnCode = DjiCameraManager_GetCameraType(position, cameraType);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            publishDjiErrorCode(returnCode);
            USER_LOG_ERROR("CameraManagerGetCameraType failed, error code: 0x%08X\r\n", returnCode);
            return returnCode;
        }
        else
        {
            USER_LOG_INFO("CameraManagerGetCameraType successfully");
        }

        return returnCode;
    }

}
