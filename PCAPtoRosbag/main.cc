#include <hesai_lidar_sdk.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <std_msgs/msg/header.hpp>
//#include "../../HesaiLidar_ROS_2.0/src/manager/source_driver_ros2.hpp"
#include <lidar_types.h>
#include <string>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sstream>
#include <hesai_ros_driver/msg/udp_frame.hpp>
#include <hesai_ros_driver/msg/udp_packet.hpp>
#include <hesai_ros_driver/msg/ptp.hpp>
#include <hesai_ros_driver/msg/firetime.hpp>
#include <hesai_ros_driver/msg/loss_packet.hpp>

#include <fstream>
#include <memory>
#include <chrono>
#include <string>
#include <functional>
#include <regex>
#include <csignal>
#include <boost/thread.hpp>

uint32_t last_frame_time = 0;
uint32_t cur_frame_time = 0;
std::string frame_id = "hesai_lidar";
rosbag2_cpp::Writer writer;
bool endflag = false;

//log info, display frame message
void lidarCallback(const LidarDecodedFrame<LidarPointXYZICRTT>  &frame) {  
  cur_frame_time = GetMicroTickCount();
  if (last_frame_time == 0) last_frame_time = GetMicroTickCount();
  if (cur_frame_time - last_frame_time > kMaxTimeInterval) {
    printf("Time between last frame and cur frame is: %u us\n", (cur_frame_time - last_frame_time));
  }
  last_frame_time = cur_frame_time;
  printf("frame:%d points:%u packet:%u start time:%lf end time:%lf\n",frame.frame_index, frame.points_num, frame.packet_num, frame.points[0].timeSecond, frame.points[frame.points_num - 1].timeSecond) ;

  //conversion here
  sensor_msgs::msg::PointCloud2 ros_msg;

  int fields = 6;
  ros_msg.fields.clear();
  ros_msg.fields.reserve(fields);
  ros_msg.width = frame.points_num; 
  ros_msg.height = 1;

  int offset = 0;
  offset = addPointField(ros_msg, "x", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "y", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "z", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "ring", 1, sensor_msgs::msg::PointField::UINT16, offset);
  offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64, offset);

  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.width * ros_msg.point_step;
  ros_msg.is_dense = false;
  ros_msg.data.resize(frame.points_num * ros_msg.point_step);
  
  sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(ros_msg, "ring");
  sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg, "timestamp");

  for (size_t i = 0; i < frame.points_num; i++)
  {
    LidarPointXYZICRTT point = frame.points[i];
    *iter_x_ = point.x;
    *iter_y_ = point.y;
    *iter_z_ = point.z;
    *iter_intensity_ = point.intensity;
    *iter_ring_ = point.ring;
    *iter_timestamp_ = point.timeSecond;
    ++iter_x_;
    ++iter_y_;
    ++iter_z_;
    ++iter_intensity_;
    ++iter_ring_;
    ++iter_timestamp_;   
  }
  
  std::cout.flush();
  auto sec = (uint64_t)floor(frame.points[0].timeSecond);
  if (sec <= std::numeric_limits<int32_t>::max()) {
    ros_msg.header.stamp.sec = (uint32_t)floor(frame.points[0].timeSecond);
    ros_msg.header.stamp.nanosec = (uint32_t)round(frame.points[0].timeNanosecond);
  } else {
    printf("does not support timestamps greater than 19 January 2038 03:14:07 (now %lf)\n", frame.points[0].timeSecond);
  }
  ros_msg.header.frame_id = frame_id;

  // Create a serialization object for PointCloud2 type
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

  // Create an empty SerializedMessage
  rclcpp::SerializedMessage serialized_msg;

  // Serialize the PointCloud2 message into the serialized_msg
  serializer.serialize_message(&ros_msg, &serialized_msg);

  writer.write(serialized_msg, "/lidar_points", "sensor_msgs/msg/PointCloud2", ros_msg.header.stamp);
}

void faultMessageCallback(const FaultMessageInfo& fault_message_info) {
  // Use fault message messages to make some judgments
  fault_message_info.Print();
  return;
}

// Determines whether the PCAP is finished playing
bool IsPlayEnded(HesaiLidarSdk<LidarPointXYZICRTT>& sdk)
{
  return sdk.lidar_ptr_->IsPlayEnded();
}

// Signal handler
void signalHandler(int signum) {
    endflag = true;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  HesaiLidarSdk<LidarPointXYZICRTT> sample;
  DriverParam param;
  param.input_param.source_type = DATA_FROM_PCAP;

  // output path
  std::string output_path = std::regex_replace(argv[1], std::regex("inputs"), "outputs");
  output_path = std::regex_replace(output_path, std::regex("hesai_lidar.pcap"), "lidar");
  
  // rosbag options
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = output_path;
  storage_options.storage_id = "mcap";
  rosbag2_cpp::ConverterOptions converter_options;
  writer.open(storage_options, converter_options);
  
  // input/correction/firetime paths
  std::string correction_path = std::regex_replace(argv[1], std::regex("inputs.*"), "PCAPtoRosbag/correction/angle_correction/XT32M2X_Angle Correction File.csv");
  std::string firetimes_path = std::regex_replace(argv[1], std::regex("inputs.*"), "PCAPtoRosbag/correction/firetime_correction/PandarXT-32M2X_Firetime Correction File.csv");
  param.input_param.pcap_path = argv[1];
  param.input_param.correction_file_path = correction_path;
  param.input_param.firetimes_path = firetimes_path;
  
  // other parameters
  param.decoder_param.distance_correction_flag = false;
  param.decoder_param.pcap_play_synchronization = false; // for speed up

  // init lidar with param
  sample.Init(param);

  // assign callback fuction
  sample.RegRecvCallback(lidarCallback);
  sample.RegRecvCallback(faultMessageCallback);

  // signal handler
  signal(SIGINT, signalHandler);

  sample.Start();

  // You can select the parameters in while():
  // 1.[IsPlayEnded(sample)]: adds the ability for the PCAP to automatically quit after playing the program
  // 2.[1                  ]: the application will not quit voluntarily
  while (!endflag && (!IsPlayEnded(sample) || GetMicroTickCount() - last_frame_time < 1000000))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  printf("The PCAP file has been converted and we will exit the program.\n");
  rclcpp::shutdown();
  return 0;
}
