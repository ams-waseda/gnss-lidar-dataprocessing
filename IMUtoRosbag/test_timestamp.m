clear;
close all;
clc;

%% IMU
bagReader = ros2bagreader("../outputs/20250710_142307_APX/imu/imu_0.mcap");
imubag = select(bagReader,"Topic","/imu");
imu = readMessages(imubag);

imusec = cell2mat(cellfun(@(m) m.header.stamp.sec,imu,'UniformOutput',false));
imunsec = cell2mat(cellfun(@(m) m.header.stamp.nanosec,imu,'UniformOutput',false));
timu = double(imusec-imusec(1))+double(imunsec)/1e9;
gyro = cell2mat(cellfun(@(m) [m.angular_velocity.x m.angular_velocity.y m.angular_velocity.z],imu,'UniformOutput',false));
acc = cell2mat(cellfun(@(m) [m.linear_acceleration.x m.linear_acceleration.y m.linear_acceleration.z],imu,'UniformOutput',false));

figure;
plot(diff(timu));
grid on;
ylim([0 0.01]);
title("Time difference (IMU)");
ylabel("s");

%% Lidar
bagReader = ros2bagreader("../outputs/20250710_142307_APX/lidar/lidar_0.mcap");
lidarbag = select(bagReader,"Topic","/lidar/points");
lidar = readMessages(lidarbag);

lidarsec = cell2mat(cellfun(@(m) m.header.stamp.sec,lidar,'UniformOutput',false));
lidarnsec = cell2mat(cellfun(@(m) m.header.stamp.nanosec,lidar,'UniformOutput',false));
tlidar = double(lidarsec-lidarsec(1))+double(lidarnsec)/1e9;

figure;
plot(diff(tlidar));
grid on;
ylim([0 0.2]);
title("Time difference (Lidar)");
ylabel("s");
