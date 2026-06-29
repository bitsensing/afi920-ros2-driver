cmake_minimum_required(VERSION 3.8)
project(afi920_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # RDI (Radar Detection Interface)
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  # SHII (Sensor Health Information Interface)
  "msg/HealthInfo.msg"
  # SPI (Sensor Performance Interface)
  "msg/SensorPose.msg"
  "msg/FovSegment.msg"
  "msg/RecognisableObject.msg"
  "msg/ReferenceTarget.msg"
  "msg/SensorPerformance.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

ament_package()
