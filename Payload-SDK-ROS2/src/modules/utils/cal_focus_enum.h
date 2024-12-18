/**
 ******************************************************************************
 * @file           : cal_focus_enum
 * @author         : gufeng
 * @brief          : None
 * @attention      : None
 * @date           : 22/7/24
 ******************************************************************************
 */

#ifndef CAL_FOCUS_ENUM_H
#define CAL_FOCUS_ENUM_H

#include <iostream>
#include <unordered_map>
#include <memory>
#include "dji_typedef.h"

struct LinearRegressionParams {
    double slope;
    double intercept;
};

struct FocusRingLimits {
    double min_value;
    double max_value;
};

class CameraControl {
public:
    CameraControl(const std::unordered_map<double, LinearRegressionParams>& params, const FocusRingLimits& limits)
            : parameters(params), limits(limits) {}

    double calculateFocusRingValue(double zoomFactor, double distance) {
        double closestDistance = findClosestKey(distance);

        if (parameters.find(closestDistance) != parameters.end()) {
            const auto& params = parameters.at(closestDistance);
            double focusRingValue = params.slope * zoomFactor + params.intercept;
            return applyLimits(focusRingValue);
        } else {
            std::cerr << "No parameters found for distance: " << closestDistance << std::endl;
            return -1;
        }
    }

private:
    [[nodiscard]] double applyLimits(double value) const {
        if (value < limits.min_value) return limits.min_value;
        if (value > limits.max_value) return limits.max_value;
        return value;
    }

    [[nodiscard]] double findClosestKey(double key) const {
        double closestKey = parameters.begin()->first;
        double minDiff = std::abs(key - closestKey);

        for (const auto& param : parameters) {
            double diff = std::abs(key - param.first);
            if (diff < minDiff) {
                minDiff = diff;
                closestKey = param.first;
            }
        }
        return closestKey;
    }

    std::unordered_map<double, LinearRegressionParams> parameters;
    FocusRingLimits limits;
};

class CameraFactory {
public:
    static std::shared_ptr<CameraControl> createCameraControl(E_DjiCameraType cameraType) {
        std::unordered_map<double, LinearRegressionParams> params;
        FocusRingLimits limits{};

        switch (cameraType) {
            case DJI_CAMERA_TYPE_M30:
                params = {
                        {5.0, {12.983823529411765, 180.76470588235293}},
                        {10.0, {5.91470588235294, 194.69117647058826}},
                        {15.0, {3.3897058823529407, 202.94117647058823}},
                        {20.0, {2.1235294117647054, 207.0808823529412}},
                        {25.0, {1.6529411764705881, 203.96323529411765}},
                        {30.0, {0.6161764705882353, 201.48529411764707}}
                };
                limits = {0, 255}; // Example range for M30
                break;
            case DJI_CAMERA_TYPE_H20:
            case DJI_CAMERA_TYPE_H20T:
                params = {
                        {5.0, {16.001470588235296, 38.04411764705878}},
                        {10.0, {9.861764705882353, 26.977941176470594}},
                        {15.0, {6.342647058823529, 41.2794117647059}},
                        {20.0, {4.567647058823529, 47.154411764705884}},
                        {25.0, {3.4897058823529408, 51.69117647058824}}
                };
                limits = {60, 158}; // Example range for H20T
                break;
            case DJI_CAMERA_TYPE_H30T:
                params = {
                        {5.0, {45.08382352941176, -32.11029411764696}},
                        {10.0, {24.279411764705877, 74.13235294117652}},
                        {15.0, {17.22058823529412, 112.61764705882351}},
                        {20.0, {12.924999999999999, 144.375}},
                        {25.0, {10.386764705882353, 164.47794117647058}},
                        {30.0, {10.219117647058823, 157.4485294117647}}
                };
                limits = {170, 253}; // Example range for H30T
                break;
                // Add more cases for other models as needed
            default:
                std::cerr << "Unknown camera type: " << cameraType << std::endl;
                return nullptr;
        }

        return std::make_shared<CameraControl>(params, limits);
    }

    static double calculateFocusRingValue(E_DjiCameraType cameraType, double zoomFactor, double distance) {
        auto cameraControl = createCameraControl(cameraType);
        if (!cameraControl) {
            std::cerr << "Failed to create CameraControl for camera type: " << cameraType << std::endl;
            return -1;
        }
        return cameraControl->calculateFocusRingValue(zoomFactor, distance);
    }
};

#endif //CAL_FOCUS_ENUM_H
