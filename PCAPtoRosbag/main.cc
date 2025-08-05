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
#include <boost/thread.hpp>

uint32_t last_frame_time = 0;
uint32_t cur_frame_time = 0;
uint32_t frame_id = 0;
rosbag2_cpp::Writer writer;

//log info, display frame message
void lidarCallback(const LidarDecodedFrame<LidarPointXYZICRTT>  &frame) {  
  cur_frame_time = GetMicroTickCount();
  if (last_frame_time == 0) last_frame_time = GetMicroTickCount();
  if (cur_frame_time - last_frame_time > kMaxTimeInterval) {
    printf("Time between last frame and cur frame is: %u us\n", (cur_frame_time - last_frame_time));
  }
  last_frame_time = cur_frame_time;
  printf("frame:%d points:%u packet:%u start time:%lf end time:%lf\n",frame.frame_index, frame.points_num, frame.packet_num, frame.points[0].timeSecond, frame.points[frame.points_num - 1].timeSecond) ;
  

  std::string framestring = std::to_string(frame_id);

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
  // printf("HesaiLidar Runing Status [standby mode:%u]  |  [speed:%u]\n", frame.work_mode, frame.spin_speed);
  // printf("frame:%d points:%u packet:%d start time:%lf end time:%lf\n",frame.frame_index, frame.points_num, frame.packet_num, frame.points[0].timestamp, frame.points[frame.points_num - 1].timestamp) ;
  std::cout.flush();
  auto sec = (uint64_t)floor(frame.points[0].timeSecond);
  if (sec <= std::numeric_limits<int32_t>::max()) {
    ros_msg.header.stamp.sec = (uint32_t)floor(frame.points[0].timeSecond);
    ros_msg.header.stamp.nanosec = (uint32_t)round(frame.points[0].timeNanosecond);
  } else {
    printf("does not support timestamps greater than 19 January 2038 03:14:07 (now %lf)\n", frame.points[0].timeSecond);
  }
  ros_msg.header.frame_id = framestring;

  ++frame_id;

  // Create a serialization object for PointCloud2 type
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

  // Create an empty SerializedMessage
  rclcpp::SerializedMessage serialized_msg;

  // Serialize the PointCloud2 message into the serialized_msg
  serializer.serialize_message(&ros_msg, &serialized_msg);

  writer.write(serialized_msg, "/point_cloud", "sensor_msgs/msg/PointCloud2", ros_msg.header.stamp);
}

void faultMessageCallback(const FaultMessageInfo& fault_message_info) {
  // Use fault message messages to make some judgments
  // fault_message_info.Print();
  return;
}

// Determines whether the PCAP is finished playing
bool IsPlayEnded(HesaiLidarSdk<LidarPointXYZICRTT>& sdk)
{
  return sdk.lidar_ptr_->IsPlayEnded();
}

int main(int argc, char *argv[])
{
//#ifndef _MSC_VER
//if (system("sudo sh -c \"echo 562144000 > /proc/sys/net/core/rmem_max\"") == -1) {
//    printf("Command execution failed!\n");
//  }
//#endif
  
  rclcpp::init(argc, argv);
  
  //writer.create_topic(
  //      {"/point_cloud", "sensor_msgs/msg/PointCloud2", "cdr", ""});
  writer.open("pointcloud");
  
  HesaiLidarSdk<LidarPointXYZICRTT> sample;
  DriverParam param;

  // assign param
  // param.decoder_param.enable_packet_loss_tool = true;
  // param.lidar_type = "XT32M2X";
  param.input_param.source_type = DATA_FROM_PCAP;
  param.input_param.pcap_path = "hesai_lidar.pcap";
  param.input_param.correction_file_path = "XT32M2X_Angle Correction File.csv";
  param.input_param.firetimes_path = "PandarXT-32M2X_Firetime Correction File.csv";

  //param.input_param.device_ip_address = "192.168.1.201";
  //param.input_param.ptc_port = 9347;
  //param.input_param.udp_port = 2368;
  // param.input_param.rs485_com = "Your serial port name for receiving point cloud";
  // param.input_param.rs232_com = "Your serial port name for sending cmd";
  //param.input_param.host_ip_address = "192.168.1.200";
  // param.input_param.multicast_ip_address = "";
  param.decoder_param.distance_correction_flag = false;
  //param.decoder_param.socket_buffer_size = 262144000;

  //init lidar with param
  sample.Init(param);

  //assign callback fuction
  sample.RegRecvCallback(lidarCallback);
  sample.RegRecvCallback(faultMessageCallback);

  sample.Start();

  // You can select the parameters in while():
  // 1.[IsPlayEnded(sample)]: adds the ability for the PCAP to automatically quit after playing the program
  // 2.[1                  ]: the application will not quit voluntarily
  while (!IsPlayEnded(sample) || GetMicroTickCount() - last_frame_time < 1000000)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  printf("The PCAP file has been converted and we will exit the program.\n");
  rclcpp::shutdown();
  return 0;
}
