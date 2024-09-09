#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, String
import subprocess

class RecordingManager(Node):
    def __init__(self):
        super().__init__('recording_manager')

        # Subscriptions
        self.subscription_status = self.create_subscription(Int64, 'recording_status', self.recording_status_callback, 10)
        self.subscription_name = self.create_subscription(String, 'recording_name', self.recording_name_callback, 10)

        self.process = None
        self.recording_started = False  # Flag to track recording status
        self.recording_name = "default_name"  # Default name for recording file if none is provided

        # Replace with your actual topic names
        self.topic_names = [
            '/zed_doorway/zed_node_doorway/left/image_rect_color',
            '/zed_doorway/zed_node_doorway/body_trk/skeletons'
        ]

    def recording_status_callback(self, msg):
        self.get_logger().info(f"Received recording status: {msg.data}")
        if msg.data == 1 and not self.recording_started:
            self.start_recording()
        elif msg.data == 0 and self.recording_started:
            self.stop_recording()

    def recording_name_callback(self, msg):
        self.recording_name = msg.data
        self.get_logger().info(f"Recording name set to: {self.recording_name}")

    def start_recording(self):
        if self.process is None:
            # Construct the command with the file name
            bag_file_name = f"rosbag2_{self.recording_name}"
            self.process = subprocess.Popen(['ros2', 'bag', 'record', '-o', bag_file_name] + self.topic_names)
            self.get_logger().info(f"Started recording {', '.join(self.topic_names)} with file name {bag_file_name}")
            self.recording_started = True

    def stop_recording(self):
        if self.process is not None:
            self.process.terminate()
            self.process.wait()
            self.process = None
            self.get_logger().info("Recording stopped.")
            self.recording_started = False
        else:
            self.get_logger().info("No recording is in progress")

def main(args=None):
    rclpy.init(args=args)
    recording_manager = RecordingManager()

    rclpy.spin(recording_manager)

    recording_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
