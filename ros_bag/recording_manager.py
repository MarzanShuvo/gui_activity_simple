#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import subprocess

class RecordingManager(Node):
    def __init__(self):
        super().__init__('recording_manager')
        self.subscription = self.create_subscription(Int64, 'recording_status', self.recording_status_callback, 10)
        self.process = None
        self.recording_started = False  # Flag to track recording status
        self.topic_names = [
            '/zed_doorway/zed_node_doorway/left/image_rect_color', 
            '/zed_doorway/zed_node_doorway/body_trk/skeletons'
        ]  # Replace with your actual topic names

    def recording_status_callback(self, msg):
        self.get_logger().info(f"Received recording status: {msg.data}")
        if msg.data == 1 and not self.recording_started:
            self.start_recording()
        elif msg.data == 0 and self.recording_started:
            self.stop_recording()

    def start_recording(self):
        if self.process is None:
            self.process = subprocess.Popen(['ros2', 'bag', 'record'] + self.topic_names)
            self.get_logger().info(f"Started recording {', '.join(self.topic_names)}")
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
