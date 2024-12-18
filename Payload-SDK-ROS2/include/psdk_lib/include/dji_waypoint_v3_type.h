/**
 ********************************************************************
 * @file    dji_waypoint_v3_type.h
 * @brief   This is the header file for "dji_waypoint_v3_type.c", defining the structure and
 * (exported) function prototypes.
 *
 * @copyright (c) 2023 SkySys. All rights reserved.
 *********************************************************************
 */

#ifndef DJI_WAYPOINT_V3_TYPE_H
#define DJI_WAYPOINT_V3_TYPE_H

#include<iostream>
#include<vector>
#include"dji_typedef.h"

using namespace std;

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
    /**
     * @brief 安全模式:
     * （M300）飞行器起飞，上升至首航点高度，再平飞至首航点。如果首航点低于起飞点，则起飞后平飞至首航点上方再下降。
     * （M30）飞行器起飞，上升至首航点高度，再平飞至首航点。如果首航点低于“安全起飞高度”，则起飞至“安全起飞高度”后，平飞至首航点上方再下降。注意“安全起飞高度”仅在飞行器未起飞时生效。
     */
    string safely       = "safely";
    string pointToPoint = "pointToPoint";
} E_flyToWaylineMode;

typedef struct{
    string goHome   = "goHome";
    string noAction = "noAction";
    string autoLand = "autoLand";
    string gotoFirstWaypoint = "gotoFirstWaypoint";
} E_finishAction;

typedef struct{
    string goContinue        = "goContinue";
    string executeLostAction = "executeLostAction";
} E_exitOnRCLost;

typedef struct{
    string goBack  = "goBack";
    string landing = "landing";
    string hover   = "hover";
} E_executeRCLostAction;

typedef struct{
    /**
     * @brief 60(机型: M300RTK), 67(机型: M30系列)
     * 必需元素
     */
    E_DjiAircraftType droneEnumValue;
    /**
     * @brief 当“飞行器机型主类型”为 “67(机型: M30系列)” 时: 0(机型: M30双光), 1(机型: M30T三光)
     * 必需元素 * 注:当 “飞行器机 型主类型” 为“67 (机型: M30系 列)”时， 该元素才 是必需。
     */
    int droneSubEnumValue;
} T_droneInfo;

typedef struct{
    /**
     * @brief 42(机型:H20), 43(机型:H20T), 50(机型:P1), 52(机型:M30双光相 机), 53(机型:M30T三光 相机), 61(机型:H20N), 90742(机型:L1)
     * 
     */
    E_DjiCameraType payloadEnumValue;
    /**
     * @brief ZENMUSE_P1: 0(LENS_24mm), 1(LENS_35mm), 2(LENS_50mm)
     * 
     */
    int payloadSubEnumValue;
    /**
     * @brief 0:飞行器1号挂载位置。M300RTK机型， 对应机身左前方。其它机型，对应主云台。 1:飞行器2号挂载位置。M300RTK机型， 对应机身右前方。 2:飞行器3号挂载位置。M300RTK机型，对应机身上方。
     * 
     */
    E_DjiMountPosition payloadPositionIndex;
} T_payloadInfo;


typedef struct{
    E_flyToWaylineMode      flyToWaylineMode;
    E_finishAction          finishAction;
    E_exitOnRCLost          exitOnRCLost;
    E_executeRCLostAction   executeRCLostAction;
    float takeOffSecurityHeight;
    float globalTransitionalSpeed;
    T_droneInfo droneInfo;
    T_payloadInfo payloadInfo;
} T_missionConfig;


typedef struct{
    string WGS84                 = "WGS84";
    string relativeToStartPoint  = "relativeToStartPoint";
    string realTimeFollowSurface = "realTimeFollowSurface";
} E_executeHeightMode;

typedef struct{
    uint16_t    templateId;
    uint16_t    waylineId;
    float       autoFlightSpeed;
    E_executeHeightMode executeHeightMode;

    float       distance;
    float       duration;
} T_Folder;

/**
 * @brief waypoints
 * 
 */
typedef struct{
    double       latitude;
    double       longitude;
} T_coordinates;

typedef struct{
    T_coordinates coordinates;
} T_Point;

typedef struct{
    string followWayline    = "followWayline";
    string manually         = "manually";
    string fixed            = "fixed";
    string smoothTransition = "smoothTransition";
} E_waypointHeadingMode;

typedef struct{
    string clockwise        = "clockwise";
    string counterClockwise = "counterClockwise";
    string followBadArc     = "followBadArc";
} E_waypointHeadingPathMode;

typedef struct{
    E_waypointHeadingMode           waypointHeadingMode;
    int                             waypointHeadingAngle;
    E_waypointHeadingPathMode       waypointHeadingPathMode;
} T_waypointHeadingParam;

typedef struct{
    string coordinateTurn                           = "coordinateTurn";
    string toPointAndStopWithDiscontinuityCurvature = "toPointAndStopWithDiscontinuityCurvature";
    string toPointAndStopWithContinuityCurvature    = "toPointAndStopWithContinuityCurvature";
    string toPointAndPassWithContinuityCurvature    = "toPointAndPassWithContinuityCurvature";
} E_waypointTurnMode;

typedef struct{
    E_waypointTurnMode  waypointTurnMode;
    float               waypointTurnDampingDist;  
} T_waypointTurnParam;

typedef struct{
    string reachPoint       = "reachPoint";
    string multipleTiming   = "multipleTiming";
    string multipleDistance = "multipleDistance";
} E_actionTriggerType;

typedef struct{
    E_actionTriggerType actionTriggerType;
    float               actionTriggerParam;
} T_actionTrigger;


typedef struct{
    string takePhoto     = "takePhoto";
    string startRecord   = "startRecord";
    string stopRecord    = "stopRecord";
    string focus         = "focus";
    string zoom          = "zoom";
    string customDirName = "customDirName";
    string gimbalRotate  = "gimbalRotate";
    string rotateYaw     = "rotateYaw";
    string hover         = "hover";
} E_actionActuatorFunc;


typedef struct{
    E_DjiMountPosition  payloadPositionIndex;
    string              fileSuffix;
} T_takePhoto;

typedef struct{
    E_DjiMountPosition  payloadPositionIndex;
    string              fileSuffix;
} T_startRecord;

typedef struct{
    E_DjiMountPosition  payloadPositionIndex;
} T_stopRecord;

typedef struct{
    E_DjiMountPosition  payloadPositionIndex;
    bool                isPointFocus;
    float               focusX;
    float               focusY;
    float               focusRegionWidth;
    float               focusRegionHeight;
} T_focus;

typedef struct{
    E_DjiMountPosition  payloadPositionIndex;
    int16_t             focalLength;
} T_zoom;

typedef struct{
    E_DjiMountPosition  payloadPositionIndex;
    string              directoryName;
} T_customDirName;

typedef struct{
    string north    = "north";
    string aircraft = "aircraft";
} E_gimbalHeadingYawBase;

typedef struct{
    string relativeAngle = "relativeAngle";
    string absoluteAngle = "absoluteAngle";
} E_gimbalRotateMode;

typedef struct{
    E_DjiMountPosition  payloadPositionIndex;
    E_gimbalHeadingYawBase gimbalHeadingYawBase;
    E_gimbalRotateMode   gimbalRotateMode;
    bool gimbalPitchRotateEnable;
    float gimbalPitchRotateAngle;

    bool gimbalRollRotateEnable;
    float gimbalRollRotateAngle;

    bool gimbalYawRotateEnable;
    float gimbalYawRotateAngle;

    bool gimbalRotateTimeEnable;
    float gimbalRotateTime;
} T_gimbalRotate;

typedef struct{
    string clockwise = "clockwise";
    string counterClockwise = "counterClockwise";
} E_aircraftPathMode;

typedef struct{
    float aircraftHeading;
    E_aircraftPathMode aircraftPathMode;
} T_rotateYaw;

typedef struct{
    T_takePhoto     takePhoto;
    T_startRecord   startRecord;
    T_stopRecord    stopRecord;
    T_focus         focus;
    T_zoom          zoom;
    T_customDirName customDirName;
    T_gimbalRotate  gimbalRotate;
    T_rotateYaw     rotateYaw;
    float           hoverTime;
} T_actionActuatorFuncParam;

typedef struct{
    uint16_t                    actionId;
    E_actionActuatorFunc        actionActuatorFunc;
    T_actionActuatorFuncParam   actionActuatorFuncParam;
} T_action;

typedef struct{
    uint16_t    actionGroupId;
    uint16_t    actionGroupStartIndex;
    uint16_t    actionGroupEndIndex;
    string      actionGroupMode = "sequence";
    T_actionTrigger actionTrigger;
    T_action        action;
} T_actionGroup;

typedef struct{
    // std::vector<string> waypoints_vector;
    T_Point     Point;
    uint16_t    index;
    float       executeHeight;
    float       waypointSpeed;
    T_waypointHeadingParam  waypointHeadingParam;
    T_waypointTurnParam     waypointTurnParam;
    bool                    useStraightLine;

    T_actionGroup actionGroup;
} T_waypoints;

#ifdef __cplusplus
}
#endif

#endif // DJI_WAYPOINT_V3_TYPE_H
