clear;
close all;
clc;

dirname = "20250925_162336_APX";

%% Read IMU data
sol = gt.Gsol("../inputs/"+dirname+"/asterx.pos");
sol.plotMap();

% Unix time
toUTCT = 18; % GPST-UTCT=18s
unix_time = posixtime(sol.time.t-seconds(toUTCT)); % sol.time is GPS Time, so it is converted to UTC Time

%% Convert to ROS bag
bagWriter = ros2bagwriter("../outputs/"+dirname+"/gnss","StorageFormat","mcap");
for i=1:1:length(unix_time)
    % output only fixed solution
    if sol.stat(i)==gt.C.SOLQ_FIX
        message2 = ros2message("geometry_msgs/PoseWithCovarianceStamped");
    
        % Header/Stamp
        message2.header.stamp.sec = int32(fix(unix_time(i)));
        message2.header.stamp.nanosec = uint32((unix_time(i)-fix(unix_time(i)))*10^9);
        message2.header.frame_id = 'gnss';
    
        % Position
        % The coordinate system is local East-North-Up
        % The origin of the coordinate system is the position of the GNSS reference station on campus
        % The coordinate system needs to be reviewed to ensure it is acceptable
        message2.pose.pose.position.x = sol.pos.east(i); % East (m)
        message2.pose.pose.position.y = sol.pos.north(i); % North (m)
        message2.pose.pose.position.z = sol.pos.up(i); % Up (m)
    
        write(bagWriter, "/gnss", message2.header.stamp, message2);
    end
end
delete(bagWriter);
