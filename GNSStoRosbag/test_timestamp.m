clear;
close all;
clc;

%% GNSS
bagReader = ros2bagreader("../outputs/20250925_162336_APX/gnss/gnss_0.mcap");
gnssbag = select(bagReader,"Topic","/gnss");
gnss = readMessages(gnssbag);

gnsssec = cell2mat(cellfun(@(m) m.header.stamp.sec,gnss,'UniformOutput',false));
gnssnsec = cell2mat(cellfun(@(m) m.header.stamp.nanosec,gnss,'UniformOutput',false));
tgnss = double(gnsssec-gnsssec(1))+double(gnssnsec)/1e9;
pos = cell2mat(cellfun(@(m) [m.pose.pose.position.x m.pose.pose.position.y m.pose.pose.position.z],gnss,'UniformOutput',false));

figure;
plot(diff(tgnss));
grid on;
title("Time difference (GNSS)");
ylabel("s");

figure;
plot(pos(:,1),pos(:,2),".-");
grid on; axis equal;