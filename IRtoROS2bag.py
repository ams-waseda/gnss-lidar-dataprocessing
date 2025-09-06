import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as PILImage, ExifTags
import numpy as np
import os
import glob
from datetime import datetime, timezone
import time
import tifffile
from builtin_interfaces.msg import Time

from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from std_msgs.msg import String
from rclpy.serialization import serialize_message

# Constants
INPUT_DIR = "IRimages"  # CHANGE THIS
ROS_TOPIC = "/image"
TIFF_TIMESTAMP_TAG = 306  # Standard "DateTime" tag

def extract_datetime_with_subseconds(tiff_page):
    EXIF_TAG_CODE = 34665  # ExifTag

    if EXIF_TAG_CODE not in tiff_page.tags:
        return None

    exif_dict = tiff_page.tags[EXIF_TAG_CODE].value
    dt_str = exif_dict.get('DateTimeOriginal')
    subseconds = exif_dict.get('SubsecTimeOriginal', '')

    if not dt_str:
        return None

    try:
        # Parse main datetime part
        dt = datetime.strptime(dt_str, "%Y:%m:%d %H:%M:%S")

        # If subseconds present, convert to microseconds and add
        if subseconds.isdigit():
            microsec = int(subseconds.ljust(6, '0'))  # pad to microseconds
            dt = dt.replace(microsecond=microsec)

        return dt
    except Exception as e:
        print(f"Error parsing datetime with subseconds: {e}")
        return None


class TIFFPublisher(Node):
    def __init__(self):
        super().__init__('tiff_publisher')
        self.serialize_message = serialize_message
        # Set up writer
        self.writer = SequentialWriter()
        self.storage_options = StorageOptions(uri='my_bag', storage_id='sqlite3')
        self.converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        self.writer.open(self.storage_options, self.converter_options)

        # Create topic
        topic_metadata = TopicMetadata(
            name=ROS_TOPIC,
            type='sensor_msgs/msg/Image',
            serialization_format='cdr'
        )
        self.writer.create_topic(topic_metadata)
        self.publisher = self.create_publisher(Image, ROS_TOPIC, 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.publish_next_image)

        self.tiff_files = self.get_tiff_files(INPUT_DIR)
        self.current_file_index = 0
        self.current_frame_index = 0
        self.current_tiff = None
        self.current_tifffile = None

        self.load_next_tiff()

    def get_tiff_files(self, directory):
        return sorted(glob.glob(os.path.join(directory, "*.tif")) +
                      glob.glob(os.path.join(directory, "*.TIFF")))

    def load_next_tiff(self):
        if self.current_file_index >= len(self.tiff_files):
            self.get_logger().info("All TIFFs published.")
            self.destroy_node()
            return

        tiff_path = self.tiff_files[self.current_file_index]
        self.current_file_index += 1
        self.current_frame_index = 0

        try:
            self.current_tifffile = tifffile.TiffFile(self.tiff_files[self.current_file_index - 1])
            self.current_tiff = PILImage.open(tiff_path)
            self.get_logger().info(f"Opened {tiff_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to open {tiff_path}: {e}")
            self.load_next_tiff()

    def publish_next_image(self):
        if not self.current_tiff:
            return

        try:
            self.current_tiff.seek(self.current_frame_index)
        except EOFError:
            self.load_next_tiff()
            return

        """Extract DateTimeOriginal from Exif as a ROS 2 Time message."""
        page = self.current_tifffile.pages[self.current_frame_index]
        dt = extract_datetime_with_subseconds(page)

        ros_time = Time()
        ros_time.sec = int(dt.timestamp())  # seconds since epoch
        # nanoseconds from microseconds (microseconds * 1000)
        ros_time.nanosec = dt.microsecond * 1000

        # Convert to OpenCV
        pil_image = self.current_tiff.copy()
        cv_image = np.array(pil_image)

        msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="mono16")
        msg.header.frame_id = "camera"
        msg.header.stamp = ros_time

        #self.publisher.publish(msg)
        serialized_msg = self.serialize_message(msg)
        self.writer.write(ROS_TOPIC,serialized_msg,ros_time.sec*1000000000+ros_time.nanosec)
        self.get_logger().info(f"Published frame {self.current_frame_index}")
        self.current_frame_index += 1


def main():
    rclpy.init()
    node = TIFFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

