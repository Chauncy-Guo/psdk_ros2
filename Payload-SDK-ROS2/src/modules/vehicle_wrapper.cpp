
#include <vehicle_wrapper.hpp>
namespace dji_psdk_ros2
{
    VehicleWrapper::VehicleWrapper(std::string config_json_path, std::string log_path) : config_json_path_(config_json_path), log_path_(log_path)
    {
        // Application application;
        applicationObj_ = make_shared<Application>(config_json_path_, log_path_);
        USER_LOG_INFO("vehicle_wrapper setup");
    }

}
