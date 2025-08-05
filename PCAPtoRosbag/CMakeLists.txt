cmake_minimum_required(VERSION 3.8.11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)
PROJECT(PCAPtoBag)
set(VERSION_MAJOR 2)
set(VERSION_MINOR 0)
set(VERSION_TINY  9)

cmake_policy(SET CMP0053 NEW)
set(CMAKE_BUILD_TYPE ON)
set(CMAKE_BUILD_TYPE Release)
add_definitions(-O3)
add_compile_options(-Wall)
# add_definitions(-DJT128_256)

# 设置生成平台为 x64
if(MSVC)
add_compile_options("/source-charset:utf-8")
set(CMAKE_GENERATOR_PLATFORM x64)
set(OPENSSL_ROOT_DIR "C:/Program Files/OpenSSL-Win64")
set(OPENSSL_INCLUDE_DIR ${OPENSSL_ROOT_DIR}/include)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)

find_package(builtin_interfaces REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(rosbag2_cpp REQUIRED)

find_package(sensor_msgs REQUIRED)

include_directories("${PROJECT_BINARY_DIR}")
find_package(OpenSSL REQUIRED)
add_definitions(-DQT_MESSAGELOGCONTEXT)
add_subdirectory(libhesai)

add_library(hesai_sdk_lib
	driver/hesai_lidar_sdk.cc
	)
target_link_libraries(hesai_sdk_lib PUBLIC 
					lidar_lib
					${OTHER_LIB})
set_target_properties(hesai_sdk_lib 
	PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 14)
target_include_directories(hesai_sdk_lib PUBLIC   
    ${OTHER_INCLUDE_DIRS}
	driver
	./
)

include_directories(
  libhesai
  libhesai/Lidar
  libhesai/UdpParser
  libhesai/UdpParser/include
  libhesai/UdpParser/src
  libhesai/UdpProtocol
  libhesai/Source/include
  libhesai/Container/include
  libhesai/Container/src
  libhesai/UdpParserGpu
  libhesai/UdpParserGpu/include
  libhesai/UdpParserGpu/src
  libhesai/PtcClient/include
  libhesai/SerialClient/include
  libhesai/PtcParser
  libhesai/PtcParser/src
  libhesai/PtcParser/include
  libhesai/Logger/include
  libhesai/include
  driver
  third_party
  hesai_ros_driver
  /opt/ros/humble/include/rclcpp
  /opt/ros/humble/include/rcl
  /opt/ros/humble/include/rcutils
  /opt/ros/humble/include/rmw
  /opt/ros/humble/include/rcl_yaml_param_parser
  /opt/ros/humble/include/rosidl_runtime_c
  /opt/ros/humble/include/rosbag2_cpp
  /opt/ros/humble/include/rosidl_typesupport_interface
  /opt/ros/humble/include/rcpputils
  /opt/ros/humble/include/builtin_interfaces
  /opt/ros/humble/include/rosidl_runtime_cpp
  /opt/ros/humble/include/tracetools
  /opt/ros/humble/include/rcl_interfaces
  /opt/ros/humble/include/libstatistics_collector
  /opt/ros/humble/include/statistics_msgs
  /opt/ros/humble/include/rosbag2_storage
  /opt/ros/humble/include/rosidl_typesupport_introspection_cpp
  /opt/ros/humble/include/std_msgs
  /opt/ros/humble/include/sensor_msgs
  /opt/ros/humble/include/geometry_msgs
  /opt/ros/humble/
  ${OPENSSL_INCLUDE_DIR}
)
#link_directories(./libhesai/PtcClient/lib)

add_executable(PCAPtoBag
  main.cc
)

target_link_directories(PCAPtoBag PUBLIC
  /opt/ros/humble/lib/
)

target_link_libraries(PCAPtoBag
  source_lib
  #container_lib
  ptcClient_lib
  serialClient_lib
  #ptcParser_lib
  log_lib
  rclcpp
  ${OPENSSL_LIBRARIES}
)

ament_target_dependencies(PCAPtoBag rclcpp
  rosbag2_cpp
  sensor_msgs
)
