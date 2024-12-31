import argparse
import glob
import os
from os.path import join

import cv2
import rclpy
from rclpy.serialization import serialize_message
from rclpy.time import Time
from sensor_msgs.msg import Image
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
import tqdm

def add_topic(writer, topic_name, message_type, serialization_format="cdr"):
    """
    Add a topic to the rosbag2 writer using TopicMetadata.
    """
    topic_metadata = TopicMetadata(
        name=topic_name,
        type=message_type,
        serialization_format=serialization_format
    )
    writer.create_topic(topic_metadata)

def create_bag_writer(output_bag_path, storage_id="sqlite3", serialization_format="cdr"):
    """
    Create a rosbag2 writer instance.
    """
    writer = SequentialWriter()

    # Storage options
    storage_options = StorageOptions(
        uri=output_bag_path,
        storage_id=storage_id
    )

    # Converter options
    converter_options = ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format
    )

    # Open writer with correct arguments
    writer.open(storage_options, converter_options)

    return writer

def extract_timestamps_from_png(file_paths):
    """
    Extract timestamps from PNG filenames in the format '<seconds><nanoseconds>.png'.
    """
    timestamps = []
    for file_path in file_paths:
        basename = os.path.basename(file_path)
        timestamp_str = os.path.splitext(basename)[0]  # Remove file extension
        seconds = int(timestamp_str[:-9])  # Extract seconds
        nanoseconds = int(timestamp_str[-9:])  # Extract nanoseconds
        timestamps.append((file_path, seconds, nanoseconds))
    return timestamps

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--rosbag_folder", required=True,
                        type=str, help="Path to the base folder containing the rosbags")
    parser.add_argument("--rgb_folder", required=True,
                        type=str, help="Path to the folder containing the 1st RGB camera images")
    parser.add_argument("--rgb_folder_2", type=str,
                        help="Path to the folder containing the 2nd RGB camera images (optional)")
    parser.add_argument("--mono_folder", required=True,
                        type=str, help="Path to the folder containing the mono images (PNG)")
    parser.add_argument("--rgb_topic", required=True, type=str,
                        help="Name of the topic for the 1st RGB images")
    parser.add_argument("--rgb_topic_2", type=str,
                        help="Name of the topic for the 2nd RGB images (optional)")
    parser.add_argument("--mono_topic", required=True, type=str,
                        help="Name of the topic for mono images")
    parser.add_argument("--downsample_rate", type=int, default=12,
                        help="Rate for downsampling the data (default: 12)")
    args = parser.parse_args()

    # List and extract timestamps for images
    mono_files = sorted(glob.glob(join(args.mono_folder, "*.png")))
    rgb_files = sorted(glob.glob(join(args.rgb_folder, "*.jpg")))
    rgb_files_2 = sorted(glob.glob(join(args.rgb_folder_2, "*.jpg"))) if args.rgb_folder_2 else None

    # Extract timestamps from PNG filenames
    mono_data = extract_timestamps_from_png(mono_files)

    # Check if the counts match
    if rgb_files_2:
        if len(mono_data) != len(rgb_files) or len(mono_data) != len(rgb_files_2):
            print(f"Mismatch between RGB1, RGB2, and Mono image counts!")
            raise ValueError("The number of images for RGB1, RGB2, and Mono cameras must match!")
    else:
        if len(mono_data) != len(rgb_files):
            print(f"Mismatch between RGB1 and Mono image counts!")
            raise ValueError("The number of images for RGB1 and Mono cameras must match!")

    # Downsample data
    downsample_rate = args.downsample_rate
    mono_data = mono_data[::downsample_rate]
    rgb_files = rgb_files[::downsample_rate]
    if rgb_files_2:
        rgb_files_2 = rgb_files_2[::downsample_rate]

    # Initialize ROS 2 node
    rclpy.init()
    node = rclpy.create_node("rosbag2_writer")

    output_bag_path = join(args.rosbag_folder, 'combined_ros2')
    writer = create_bag_writer(output_bag_path)

    # Add topics
    add_topic(writer, args.rgb_topic, "sensor_msgs/msg/Image")
    if rgb_files_2:
        add_topic(writer, args.rgb_topic_2, "sensor_msgs/msg/Image")
    add_topic(writer, args.mono_topic, "sensor_msgs/msg/Image")

    pbar = tqdm.tqdm(total=len(mono_data))
    for i, (mono_path, seconds, nanoseconds) in enumerate(mono_data):
        assert os.path.exists(rgb_files[i]) and os.path.exists(mono_path)
        rgb_path = rgb_files[i]
        rgb_path_2 = rgb_files_2[i] if rgb_files_2 else None

        try:
            # Load images
            rgb_img = cv2.imread(rgb_path, cv2.IMREAD_COLOR)  # Load 1st RGB image
            mono_img = cv2.imread(mono_path, cv2.IMREAD_GRAYSCALE)  # Load Mono image
            rgb_img_2 = cv2.imread(rgb_path_2, cv2.IMREAD_COLOR) if rgb_path_2 else None  # Load 2nd RGB image (optional)

            # Create ROS 2 Time object
            mono_stamp_ros = Time(seconds=seconds, nanoseconds=nanoseconds)
            rgb_stamp_ros = Time(seconds=seconds, nanoseconds=nanoseconds+1)
            rgb_stamp_ros_2 = Time(seconds=seconds, nanoseconds=nanoseconds+2) if rgb_img_2 else None

            # Create and write 1st RGB Image message
            rgb_msg = Image()
            rgb_msg.header.stamp = rgb_stamp_ros.to_msg()
            rgb_msg.height = rgb_img.shape[0]
            rgb_msg.width = rgb_img.shape[1]
            rgb_msg.step = rgb_img.shape[1] * 3  # RGB = 3 bytes per pixel
            rgb_msg.encoding = "rgb8"
            rgb_msg.data = rgb_img.tobytes()
            writer.write(args.rgb_topic, serialize_message(rgb_msg), rgb_stamp_ros.nanoseconds)

            # Create and write Mono Image message
            mono_msg = Image()
            mono_msg.header.stamp = mono_stamp_ros.to_msg()
            mono_msg.height = mono_img.shape[0]
            mono_msg.width = mono_img.shape[1]
            mono_msg.step = mono_img.shape[1]  # Mono = 1 byte per pixel
            mono_msg.encoding = "mono8"
            mono_msg.data = mono_img.tobytes()
            writer.write(args.mono_topic, serialize_message(mono_msg), mono_stamp_ros.nanoseconds)

            # Create and write 2nd RGB Image message (if available)
            if rgb_img_2:
                rgb_msg_2 = Image()
                rgb_msg_2.header.stamp = rgb_stamp_ros_2.to_msg()
                rgb_msg_2.height = rgb_img_2.shape[0]
                rgb_msg_2.width = rgb_img_2.shape[1]
                rgb_msg_2.step = rgb_img_2.shape[1] * 3  # RGB = 3 bytes per pixel
                rgb_msg_2.encoding = "rgb8"
                rgb_msg_2.data = rgb_img_2.tobytes()
                writer.write(args.rgb_topic_2, serialize_message(rgb_msg_2), rgb_stamp_ros_2.nanoseconds)

            pbar.update(1)
        except Exception as e:
            print(f"Error processing files {rgb_path}, {rgb_path_2}, and {mono_path}: {e}")

    # Cleanup
    writer.close()
    rclpy.shutdown()
