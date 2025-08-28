clear;
close all;
clc;

dirname = "20250710_142307_APX";

%% Read IMU data
imu = readmatrix("../inputs/"+dirname+"/asterx.sbf_SBF_ExtSensorMeas1.txt");

% Accelaration (m/s^2)
idxacc = imu(:,6)==0;
tacc = gt.Gtime(imu(idxacc,1)/1000,imu(idxacc,2));
acc = imu(idxacc,13:15);

% Angular rate (deg/s)
idxgyro = imu(:,6)==1;
tgyro = gt.Gtime(imu(idxgyro,1)/1000,imu(idxgyro,2));
gyro = imu(idxgyro,16:18);

% Unix time
toUTCT = 18; % GPST-UTCT=18s
unix_time = posixtime(tacc.t-seconds(toUTCT)); % tacc/tgyro is GPS Time, so it is converted to UTC Time

%% Plot and Check
% Check for missing data
assert(all(diff(tacc.t)==seconds(0.005)));
assert(all(diff(tgyro.t)==seconds(0.005)));
assert(all(tgyro.t==tacc.t));

% Plot
tiledlayout(3,1,"TileSpacing","compact");
nexttile;
plot(acc); grid on;
title("Acceleration");
ylabel("m/s^2");
legend("x","y","z");
nexttile;
plot(gyro); grid on;
title("Angular rate");
ylabel("deg/s");
legend("x","y","z");
nexttile;
plot(diff(unix_time));
grid on;
ylim([0 0.01]);
title("Time difference");
ylabel("s");

%% Convert to ROS bag
bagWriter = ros2bagwriter("../outputs/"+dirname+"/imu","StorageFormat","mcap");
for i=1:1:length(unix_time)
    message2 = ros2message("sensor_msgs/Imu");

    % Header/Stamp
    message2.header.stamp.sec = int32(fix(unix_time(i)));
    message2.header.stamp.nanosec = uint32((unix_time(i)-fix(unix_time(i)))*10^9);
    message2.header.frame_id = 'imu';

    % Angular rate
    message2.angular_velocity.x = deg2rad(gyro(i,1)); % rad/s
    message2.angular_velocity.y = deg2rad(gyro(i,2));
    message2.angular_velocity.z = deg2rad(gyro(i,3));

    % Accelaration
    message2.linear_acceleration.x = acc(i,1); % m/s^2
    message2.linear_acceleration.y = acc(i,2);
    message2.linear_acceleration.z = acc(i,3);

    write(bagWriter, "/imu", message2.header.stamp, message2);
end
delete(bagWriter);
