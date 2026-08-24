from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore
import numpy as np

#PARAMETERS
inputfile = input("TXT file: ")
bagpath = input("ROSbag path: ")

typestore = get_typestore(Stores.LATEST)
IMU = typestore.types['sensor_msgs/msg/Imu']
Vector3 = typestore.types['geometry_msgs/msg/Vector3']
Header = typestore.types['std_msgs/msg/Header']
Time = typestore.types['builtin_interfaces/msg/Time']
Quaternion = typestore.types["geometry_msgs/msg/Quaternion"]

#convert IMU txt to array
data = np.loadtxt(inputfile, dtype=str, skiprows=2, delimiter=',')
#initialize arrays
timestamps = []
xaccel = []
yaccel = []
zaccel = []
xgyro = []
ygyro = []
zgyro = []

for row in data:
    try:
        #The GPS time is in 0.001 seconds, so divide by 1000 for real seconds
        GPSsecond = float(row[0])/1000
        GPSweek = float(row[1])
        #Half of the timestamps are duplicates, so we'll skip those
        if int(row[5]) == 0:
            GPSt = GPSsecond + (GPSweek * 604800)
            #As of 2026 there are 18 leap-seconds
            unixt = GPSt + 315964800 - 18
            #Timestamps should be unix NANOSECONDS
            timestamps.append(unixt*1_000_000_000)
    except:
        #In case the script attempts to read the titles or other non-data rows
        #These SHOULD be skipped in normal operation
        print("Timestamp failed to read, skipping...")
    else:
        #write data
        #NOTE: Index 5 indicates "type" where linear acceleration == 0 and gyro == 1
        #linacc is index 12 13 14
        #gyro is index 15 16 17
        if int(row[5]) == 0:
            xaccel.append(float(row[12]))
            yaccel.append(float(row[13]))
            zaccel.append(float(row[14]))
        elif int(row[5]) == 1:
            xgyro.append(float(row[15]))
            ygyro.append(float(row[16]))
            zgyro.append(float(row[17]))

processeddata = np.column_stack([timestamps, xaccel, yaccel, zaccel, xgyro, ygyro, zgyro])

#with Writer(bagpath, version=2, storage_plugin='mcap') as writer:
with Writer(bagpath, version=2) as writer:
    #set topic
    topic = '/imu'
    #set message type
    msgtype = IMU.__msgtype__
    #add connection
    connection = writer.add_connection(topic, msgtype, typestore=typestore)

    for row in processeddata:
        #form message
        message = IMU(
            header=Header(
                stamp=Time(sec=int(row[0]/1_000_000_000), nanosec=int(row[0] % 1_000_000_000)),
                frame_id="imu",
            ),

            orientation=Quaternion(
                x=0.0,
                y=0.0,
                z=0.0,
                w=1.0,
            ),

            orientation_covariance=np.array(
                [0.0] * 9,
                dtype=np.float64
            ),

            angular_velocity=Vector3(
                x=row[4],
                y=row[5],
                z=row[6],
            ),

            angular_velocity_covariance=np.array(
                [0.0] * 9,
                dtype=np.float64
            ),

            linear_acceleration=Vector3(
                x=row[1],
                y=row[2],
                z=row[3],
            ),

            linear_acceleration_covariance=np.array(
                [0.0] * 9,
                dtype=np.float64
            ),
        )
        #write message
        msgdata = typestore.serialize_cdr(message, connection.msgtype)
        writer.write(connection, int(row[0])/1_000_000_000, msgdata)
