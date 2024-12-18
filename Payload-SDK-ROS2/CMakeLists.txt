cmake_minimum_required(VERSION 3.5)
project(dji_psdk_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)


if (NOT USE_SYSTEM_ARCH)
    # select use platform 'LINUX' here
    set(USE_SYSTEM_ARCH LINUX)
endif ()

if (USE_SYSTEM_ARCH MATCHES LINUX)
    add_definitions(-DSYSTEM_ARCH_LINUX)

    add_subdirectory(src)
    
    execute_process(COMMAND uname -m OUTPUT_VARIABLE DEVICE_SYSTEM_ID)
    if (DEVICE_SYSTEM_ID MATCHES x86_64)
        set(LIBRARY_PATH include/psdk_lib/lib/x86_64-linux-gnu-gcc)
    elseif (DEVICE_SYSTEM_ID MATCHES aarch64)
        set(LIBRARY_PATH include/psdk_lib/lib/aarch64-linux-gnu-gcc)
    else ()
        message(FATAL_ERROR "FATAL: Please confirm your platform.")
    endif ()

    install(FILES ${LIBRARY_PATH}/libpayloadsdk.a
            DESTINATION "${CMAKE_INSTALL_PREFIX}/lib"
            )

    install(DIRECTORY include/psdk_lib/include
            DESTINATION "${CMAKE_INSTALL_PREFIX}"
            )
endif ()
#end add


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()