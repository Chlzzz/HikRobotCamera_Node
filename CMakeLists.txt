cmake_minimum_required(VERSION 3.8)
project(hik_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

SET(CMAKE_BUILD_TYPE "Debug")  # 定义编译类型
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb") # 定义Debug编译参数
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall") # 定义Release编译参数


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
#find_package(camera_calibration_parsers REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED)

include_directories(
  include
  /opt/MVS/include/
 )
 
LINK_DIRECTORIES(
  /opt/MVS/lib/64/
)

add_library(HikCamera SHARED
  src/hik_camera.cpp)
target_compile_definitions(HikCamera
  PRIVATE "COMPOSITION_BUILDING_DLL")


rclcpp_components_register_nodes(HikCamera "camera::CameraNode")
set(node_plugins "${node_plugins}camera::CameraNode;$<TARGET_FILE:HikCamera>\n")

add_executable(${PROJECT_NAME}
  src/main.cpp
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  cv_bridge
  sensor_msgs
  camera_info_manager
  image_transport
)

ament_target_dependencies(HikCamera
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME}
  HikCamera
  GCBase_gcc485_v3_0
  MvCameraControl
  MVGigEVisionSDK
  log4cpp_gcc485_v3_0
  MVRender
  Log_gcc485_v3_0
  MvUsb3vTL
  MediaProcess
)

install(TARGETS
  HikCamera
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
