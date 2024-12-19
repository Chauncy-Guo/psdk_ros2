/**
 ********************************************************************
 * @file    test_liveview.cpp
 * @brief
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "test_liveview.hpp"

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/
std::map<::E_DjiLiveViewCameraPosition, DJICameraStreamDecoder *> streamDecoder;

/* Exported values ------------------------------------------------------------*/
// std::string LiveviewSample::s_fpvCameraStreamFilePath;
// std::string LiveviewSample::s_payloadCameraStreamFilePath;

static std::string s_fpvCameraStreamFilePath;
static std::string s_payloadCameraStreamFilePath;

/* Private functions declaration ---------------------------------------------*/
static void LiveviewConvertH264ToRgbCallback(E_DjiLiveViewCameraPosition position, const uint8_t *buf, uint32_t bufLen);

static void FpvCameraStreamCallback(E_DjiLiveViewCameraPosition position, const uint8_t *buf, uint32_t bufLen);
static void PayloadCameraStreamCallback(E_DjiLiveViewCameraPosition position, const uint8_t *buf, uint32_t bufLen);

/* Exported functions definition ---------------------------------------------*/
LiveviewSample::LiveviewSample()
{
    T_DjiReturnCode returnCode;

    returnCode = DjiLiveview_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        perror("Liveview init failed");
    }

    streamDecoder = {
        {DJI_LIVEVIEW_CAMERA_POSITION_FPV,  (new DJICameraStreamDecoder())},
        {DJI_LIVEVIEW_CAMERA_POSITION_NO_1, (new DJICameraStreamDecoder())},
        {DJI_LIVEVIEW_CAMERA_POSITION_NO_2, (new DJICameraStreamDecoder())},
        {DJI_LIVEVIEW_CAMERA_POSITION_NO_3, (new DJICameraStreamDecoder())},
    };
}

LiveviewSample::~LiveviewSample()
{
    T_DjiReturnCode returnCode;

    returnCode = DjiLiveview_Deinit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        perror("Liveview deinit failed");
    }

    for (auto pair : streamDecoder) {
        if (pair.second) {
            delete pair.second;
        }
    }
}

T_DjiReturnCode LiveviewSample::StartFpvCameraStream(CameraImageCallback callback, void *userData)
{
    auto deocder = streamDecoder.find(DJI_LIVEVIEW_CAMERA_POSITION_FPV);

    if ((deocder != streamDecoder.end()) && deocder->second) {
        deocder->second->init();
        deocder->second->registerCallback(callback, userData);

        return DjiLiveview_StartH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_FPV, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT,
                                           LiveviewConvertH264ToRgbCallback);
    } else {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
}

T_DjiReturnCode LiveviewSample::StartMainCameraStream(CameraImageCallback callback, void *userData)
{
    auto deocder = streamDecoder.find(DJI_LIVEVIEW_CAMERA_POSITION_NO_1);

    if ((deocder != streamDecoder.end()) && deocder->second) {
        deocder->second->init();
        deocder->second->registerCallback(callback, userData);

        return DjiLiveview_StartH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_NO_1, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT,
                                           LiveviewConvertH264ToRgbCallback);
    } else {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
}

T_DjiReturnCode LiveviewSample::StartViceCameraStream(CameraImageCallback callback, void *userData)
{
    auto deocder = streamDecoder.find(DJI_LIVEVIEW_CAMERA_POSITION_NO_2);

    if ((deocder != streamDecoder.end()) && deocder->second) {
        deocder->second->init();
        deocder->second->registerCallback(callback, userData);

        return DjiLiveview_StartH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_NO_2, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT,
                                           LiveviewConvertH264ToRgbCallback);
    } else {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
}

T_DjiReturnCode LiveviewSample::StartTopCameraStream(CameraImageCallback callback, void *userData)
{
    auto deocder = streamDecoder.find(DJI_LIVEVIEW_CAMERA_POSITION_NO_3);

    if ((deocder != streamDecoder.end()) && deocder->second) {
        deocder->second->init();
        deocder->second->registerCallback(callback, userData);

        return DjiLiveview_StartH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_NO_3, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT,
                                           LiveviewConvertH264ToRgbCallback);
    } else {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
}

T_DjiReturnCode LiveviewSample::StopFpvCameraStream()
{
    T_DjiReturnCode returnCode;

    returnCode = DjiLiveview_StopH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_FPV, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return returnCode;
    }

    auto deocder = streamDecoder.find(DJI_LIVEVIEW_CAMERA_POSITION_FPV);
    if ((deocder != streamDecoder.end()) && deocder->second) {
        deocder->second->cleanup();
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode LiveviewSample::StopMainCameraStream()
{
    T_DjiReturnCode returnCode;

    returnCode = DjiLiveview_StopH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_NO_1, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return returnCode;
    }

    auto deocder = streamDecoder.find(DJI_LIVEVIEW_CAMERA_POSITION_NO_1);
    if ((deocder != streamDecoder.end()) && deocder->second) {
        deocder->second->cleanup();
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode LiveviewSample::StopViceCameraStream()
{
    T_DjiReturnCode returnCode;

    returnCode = DjiLiveview_StopH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_NO_2, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return returnCode;
    }

    auto deocder = streamDecoder.find(DJI_LIVEVIEW_CAMERA_POSITION_NO_2);
    if ((deocder != streamDecoder.end()) && deocder->second) {
        deocder->second->cleanup();
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode LiveviewSample::StopTopCameraStream()
{
    T_DjiReturnCode returnCode;

    returnCode = DjiLiveview_StopH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_NO_3, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return returnCode;
    }

    auto deocder = streamDecoder.find(DJI_LIVEVIEW_CAMERA_POSITION_NO_3);
    if ((deocder != streamDecoder.end()) && deocder->second) {
        deocder->second->cleanup();
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}


std::string     LiveviewSample::BuildCameraStreamFilePath(std::string mountPosition){
    time_t currentTime = time(NULL);
    struct tm *localTime = NULL;
    std::string CameraStreamFilePath;
    char s_CameraStreamFilePath[256];

    struct timeval tpend;
    gettimeofday(&tpend,NULL);//获取ms级别的时间
    int milliseconds = tpend.tv_usec/1000;

    localTime = localtime(&currentTime);
    sprintf(s_CameraStreamFilePath, "%s_stream_%04d%02d%02d_%02d-%02d-%02d-%03d.h264",
            mountPosition.c_str(), localTime->tm_year + 1900, localTime->tm_mon + 1, localTime->tm_mday,
            localTime->tm_hour, localTime->tm_min, localTime->tm_sec, milliseconds);

    CameraStreamFilePath = s_CameraStreamFilePath;
    return CameraStreamFilePath;
}

T_DjiReturnCode LiveviewSample::StartFpvCameraSaveH264File(std::string CameraStreamFilePath){
    T_DjiReturnCode returnCode;
    s_fpvCameraStreamFilePath = CameraStreamFilePath;

    returnCode = DjiLiveview_StartH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_FPV, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT,
                                             FpvCameraStreamCallback);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request save h264 file of fpv failed, error code: 0x%08X", returnCode);
    } else{
        USER_LOG_INFO("Request save h264 file of fpv success");
    }
    return returnCode;
}

T_DjiReturnCode LiveviewSample::StartMainCameraSaveH264File(E_DjiLiveViewCameraSource request_source, std::string CameraStreamFilePath){
    T_DjiReturnCode returnCode;
    s_payloadCameraStreamFilePath = CameraStreamFilePath;
    
    returnCode = DjiLiveview_StartH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_NO_1, request_source,
                                             PayloadCameraStreamCallback);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request save h264 file of main camera failed, error code: 0x%08X", returnCode);
    } else{
        USER_LOG_INFO("Request save h264 file of main camera success");
    }
    return returnCode;
}

T_DjiReturnCode LiveviewSample::StartViceCameraSaveH264File(E_DjiLiveViewCameraSource request_source, std::string CameraStreamFilePath){
    T_DjiReturnCode returnCode;
    s_payloadCameraStreamFilePath = CameraStreamFilePath;
    
    returnCode = DjiLiveview_StartH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_NO_2, request_source,
                                             PayloadCameraStreamCallback);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request save h264 file of vice camera failed, error code: 0x%08X", returnCode);
    } else{
        USER_LOG_INFO("Request save h264 file of vice camera success");
    }
    return returnCode;
}

T_DjiReturnCode LiveviewSample::StartTopCameraSaveH264File(E_DjiLiveViewCameraSource request_source, std::string CameraStreamFilePath){
    T_DjiReturnCode returnCode;
    s_payloadCameraStreamFilePath = CameraStreamFilePath;
    
    returnCode = DjiLiveview_StartH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_NO_3, request_source,
                                             PayloadCameraStreamCallback);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request save h264 file of top camera failed, error code: 0x%08X", returnCode);
    } else{
        USER_LOG_INFO("Request save h264 file of top camera success");
    }
    return returnCode;
}

T_DjiReturnCode LiveviewSample::StopFpvCameraSaveH264File(){
    T_DjiReturnCode returnCode;
    returnCode = DjiLiveview_StopH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_FPV, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request to stop save h264 file of fpv failed, error code: 0x%08X", returnCode);
    } else{
        USER_LOG_INFO("Request to stop save h264 file of fpv success");
    }
    return returnCode;
}

T_DjiReturnCode LiveviewSample::StopMainCameraSaveH264File(E_DjiLiveViewCameraSource request_source){
    T_DjiReturnCode returnCode;
    returnCode = DjiLiveview_StopH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_NO_1, request_source);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request to stop save h264 file of main camera failed, error code: 0x%08X", returnCode);
    } else{
        USER_LOG_INFO("Request to stop save h264 file of main camera success");
    }
    return returnCode;
}

T_DjiReturnCode LiveviewSample::StopViceCameraSaveH264File(E_DjiLiveViewCameraSource request_source){
    T_DjiReturnCode returnCode;
    returnCode = DjiLiveview_StopH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_NO_2, request_source);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request to stop save h264 file of vice camera failed, error code: 0x%08X", returnCode);
    } else{
        USER_LOG_INFO("Request to stop save h264 file of vice camera success");
    }
    return returnCode;
}

T_DjiReturnCode LiveviewSample::StopTopCameraSaveH264File(E_DjiLiveViewCameraSource request_source){
    T_DjiReturnCode returnCode;
    returnCode = DjiLiveview_StopH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_NO_3, request_source);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request to stop save h264 file of top camera failed, error code: 0x%08X", returnCode);
    } else{
        USER_LOG_INFO("Request to stop save h264 file of top camera success");
    }
    return returnCode;
}


/* Private functions definition-----------------------------------------------*/
static void LiveviewConvertH264ToRgbCallback(E_DjiLiveViewCameraPosition position, const uint8_t *buf, uint32_t bufLen)
{
    auto deocder = streamDecoder.find(position);
    if ((deocder != streamDecoder.end()) && deocder->second) {
        deocder->second->decodeBuffer(buf, bufLen);
    }
}

static void FpvCameraStreamCallback(E_DjiLiveViewCameraPosition position, const uint8_t *buf,
                                            uint32_t bufLen)
{
    FILE *fp = NULL;
    size_t size;

    fp = fopen(s_fpvCameraStreamFilePath.c_str(), "ab+");
    if (fp == NULL) {
        printf("fopen failed!\n");
        return;
    }

    size = fwrite(buf, 1, bufLen, fp);
    if (size != bufLen) {
        fclose(fp);
        return;
    }

    fflush(fp);
    fclose(fp);
}

static void PayloadCameraStreamCallback(E_DjiLiveViewCameraPosition position, const uint8_t *buf,
                                                uint32_t bufLen)
{
    FILE *fp = NULL;
    size_t size;

    fp = fopen(s_payloadCameraStreamFilePath.c_str(), "ab+");
    if (fp == NULL) {
        printf("fopen failed!\n");
        return;
    }

    size = fwrite(buf, 1, bufLen, fp);
    if (size != bufLen) {
        fclose(fp);
        return;
    }

    fflush(fp);
    fclose(fp);
}
/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/