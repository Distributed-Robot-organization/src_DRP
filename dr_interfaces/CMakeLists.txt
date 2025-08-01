cmake_minimum_required(VERSION 3.8)
project(dr_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedSurfaces.msg"
  "msg/DetectedObjects.msg"
  "msg/SegmentationResult.msg"
  "msg/Yolov8Segmentation.msg"
  "msg/ObjectDetectionBox.msg"
  "msg/ObjectDetectionResult.msg"
  "msg/ObjectCount.msg"
  "msg/ObjectInfo.msg"
  "msg/ObjectInfoArray.msg"

  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_package()
