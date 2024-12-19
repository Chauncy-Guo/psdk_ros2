set(CMAKE_C_FLAGS "-pthread -std=gnu99")
set(CMAKE_EXE_LINKER_FLAGS "-pthread")
set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")
add_definitions(-D_GNU_SOURCE)
# add_definitions(-DCMAKE_BUILD_TYPE=Debug)


set(COMMON_CXX_FLAGS "-std=c++14 -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_CXX_FLAGS} -fprofile-arcs -ftest-coverage -Wno-deprecated-declarations")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs -ftest-coverage -lgcov")


execute_process(COMMAND uname -m
        OUTPUT_VARIABLE DEVICE_SYSTEM_ID)
if (DEVICE_SYSTEM_ID MATCHES x86_64)
    set(TOOLCHAIN_NAME x86_64-linux-gnu-gcc)
    add_definitions(-DPLATFORM_ARCH_x86_64=1)
elseif (DEVICE_SYSTEM_ID MATCHES aarch64)
    set(TOOLCHAIN_NAME aarch64-linux-gnu-gcc)
    add_definitions(-DPLATFORM_ARCH_aarch64=1)
else ()
    message(FATAL_ERROR "FATAL: Please confirm your platform.")
endif ()



include_directories(../include)
include_directories(../include/common)
include_directories(../include/dji_psdk_ros2)
include_directories(../include/psdk_lib)

include_directories(modules)
include_directories(modules/camera)
# include_directories(modules/hms)
include_directories(modules/utils)
include_directories(modules/liveview)

file(GLOB_RECURSE PSDK_NODE dji_psdk_vehicle_node.cpp)

file(GLOB_RECURSE MODULE_SAMPLE_SRC
            modules/*.c*
            modules/camera/*.c*
            modules/liveview/*.c*
        )
file(GLOB_RECURSE MODULE_COMMON_SRC ../include/common/*.c*)
file(GLOB_RECURSE MODULE_HAL_SRC modules/hal/*.c*)
file(GLOB_RECURSE MODULE_APP_SRC modules/application.c*)



set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../include/common/3rdparty)
link_directories(${CMAKE_CURRENT_LIST_DIR}/../include/psdk_lib/lib/${TOOLCHAIN_NAME})
link_libraries(${CMAKE_CURRENT_LIST_DIR}/../include/psdk_lib/lib/${TOOLCHAIN_NAME}/libpayloadsdk.a -lstdc++ /usr/lib/aarch64-linux-gnu/libboost_thread.a /usr/lib/aarch64-linux-gnu/libboost_date_time.a)


add_executable(${PROJECT_NAME}
        ${MODULE_UNIT_TEST_SAMPLE_SRC}
        ${MODULE_APP_SRC}
        ${MODULE_SAMPLE_SRC}
        ${MODULE_COMMON_SRC}
        ${MODULE_HAL_SRC}
        ${PSDK_NODE}
        )

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs 
geometry_msgs
nav_msgs
nmea_msgs
message_filters
sensor_msgs
cv_bridge
)

install(TARGETS
    ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

# Try to see if user has OpenCV installed因为额
# if yes, default callback will display the image
find_package(OpenCV QUIET)
if (OpenCV_FOUND)
    message("\n${PROJECT_NAME}...")
    message(STATUS "Found OpenCV installed in the system, will use it to display image in AdvancedSensing APIs")
    message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS " - Libraries: ${OpenCV_LIBRARIES}")
    add_definitions(-DOPEN_CV_INSTALLED)
    execute_process(COMMAND opencv_version OUTPUT_VARIABLE OPENCV_VERSION)
    if (${OPENCV_VERSION} STRLESS "4.0.0")
        add_definitions(-DOPEN_CV_VERSION_3)
    else()
        add_definitions(-DOPEN_CV_VERSION_4)
    endif()
else ()
    message(STATUS "Did not find OpenCV in the system, image data is inside RecvContainer as raw data")
endif ()

find_package(OpenCV QUIET)
if (OpenCV_FOUND)
    message("\n${PROJECT_NAME}...")
    message(STATUS "Found OpenCV installed in the system, will use it to display image in AdvancedSensing APIs")
    message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS " - Libraries: ${OpenCV_LIBRARIES}")
    add_definitions(-DOPEN_CV_INSTALLED)
    execute_process(COMMAND opencv_version OUTPUT_VARIABLE OPENCV_VERSION)
    if (${OPENCV_VERSION} STRLESS "4.0.0")
        add_definitions(-DOPEN_CV_VERSION_3)
    else()
        add_definitions(-DOPEN_CV_VERSION_4)
    endif()
else ()
    message(STATUS "Did not find OpenCV in the system, image data is inside RecvContainer as raw data")
endif ()

find_package(FFMPEG REQUIRED)
if (FFMPEG_FOUND)
    message(STATUS "Found FFMPEG installed in the system")
    message(STATUS " - Includes: ${FFMPEG_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${FFMPEG_LIBRARIES}")

    EXECUTE_PROCESS(COMMAND ffmpeg -version
            OUTPUT_VARIABLE ffmpeg_version_output
            OUTPUT_STRIP_TRAILING_WHITESPACE
            )
    string(REGEX MATCH "version.*Copyright" ffmpeg_version_line ${ffmpeg_version_output})
    string(REGEX MATCH " .* " ffmpeg_version ${ffmpeg_version_line})
    string(REGEX MATCH "^ 5.*$" ffmpeg_major_version ${ffmpeg_version})

    if (HEAD${ffmpeg_major_version} STREQUAL "HEAD")
        message(STATUS " - Version: ${ffmpeg_version}")
    else ()
        message(FATAL_ERROR " - Not support FFMPEG version: ${ffmpeg_major_version}, please install 4.x.x instead.")
    endif ()

    target_link_libraries(${PROJECT_NAME} ${FFMPEG_LIBRARIES})
    include_directories(${FFMPEG_INCLUDE_DIR})
    add_definitions(-DFFMPEG_INSTALLED)
else ()
    message(STATUS "Cannot Find FFMPEG")
endif (FFMPEG_FOUND)
include_directories(${CMAKE_CURRENT_LIST_DIR}/../include/psdk_lib/include)

find_package(OPUS REQUIRED)
if (OPUS_FOUND)
    message(STATUS "Found OPUS installed in the system")
    message(STATUS " - Includes: ${OPUS_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${OPUS_LIBRARY}")

    add_definitions(-DOPUS_INSTALLED)
    target_link_libraries(${PROJECT_NAME} /usr/local/lib/libopus.a)
else ()
    message(STATUS "Cannot Find OPUS")
endif (OPUS_FOUND)

find_package(LIBUSB REQUIRED)
if (LIBUSB_FOUND)
    message(STATUS "Found LIBUSB installed in the system")
    message(STATUS " - Includes: ${LIBUSB_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${LIBUSB_LIBRARY}")

    add_definitions(-DLIBUSB_INSTALLED)
    target_link_libraries(${PROJECT_NAME} usb-1.0)
else ()
    message(STATUS "Cannot Find LIBUSB")
endif (LIBUSB_FOUND)

target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
if (OpenCV_FOUND)
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
endif ()


# if (NOT EXECUTABLE_OUTPUT_PATH)
#     set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
# endif ()

# target_link_libraries(${PROJECT_NAME} m)

# add_custom_command(TARGET ${PROJECT_NAME}
#         PRE_LINK COMMAND cmake ..
#         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
if (OpenCV_FOUND)
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
endif ()
