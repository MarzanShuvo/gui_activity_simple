#!/usr/bin/env python3
# pip install rclpy PyQt5
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int64  # Import the message type
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QSizePolicy, QSpacerItem
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import QSize, Qt, QTimer

class BagRecorder(Node):
    def __init__(self, info_label, topic_names):
        super().__init__('bag_recorder')
        self.info_label = info_label
        self.publisher_ = self.create_publisher(Int64, 'recording_status', 10)  # Create the publisher
        self.current_status = 0  # Initialize the current status to 0
        self.timer = self.create_timer(1.0, self.publish_status)  # Set up the timer to call publish_status every second
        self.topic_names = topic_names

    def start_recording(self):
        if self.topics_available():
            self.current_status = 1  # Update the current status to 1
            self.info_label.setText("Started recording")
            self.get_logger().info("Started recording")
        else:
            self.info_label.setText("One or more topics not available")
            self.get_logger().info("One or more topics not available")

    def stop_recording(self):
        self.current_status = 0  # Update the current status to 0
        self.info_label.setText("Stopped recording")
        self.get_logger().info("Stopped recording")

    def publish_status(self):
        msg = Int64()
        msg.data = self.current_status
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published status: {self.current_status}")

    def topics_available(self):
        available_topics = self.get_topic_names_and_types()
        available_topic_names = [name for name, _ in available_topics]
        for topic in self.topic_names:
            if topic not in available_topic_names:
                self.get_logger().info(f"Topic {topic} not found")
                return False
        self.get_logger().info("All topics are available")
        return True

class MainWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node

        self.setWindowTitle("ROS 2 Bag Recorder")
        self.setGeometry(100, 100, 400, 300)

        self.layout = QVBoxLayout()

        self.button_layout = QHBoxLayout()

        self.start_button = QPushButton("")
        self.start_button.setIcon(QIcon('start_button.png'))  # Set the start button icon
        self.start_button.clicked.connect(self.start_recording)
        self.start_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.start_button.setMaximumSize(200, 200)
        self.button_layout.addWidget(self.start_button)

        self.stop_button = QPushButton("")
        self.stop_button.setIcon(QIcon('stop_button.png'))  # Set the stop button icon
        self.stop_button.clicked.connect(self.stop_recording)
        self.stop_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.stop_button.setMaximumSize(200, 200)
        self.button_layout.addWidget(self.stop_button)

        # Center the buttons vertically and horizontally
        self.layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        self.layout.addLayout(self.button_layout)
        self.layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))

        self.info_label = QLabel("Ready")
        self.layout.addWidget(self.info_label, alignment=Qt.AlignCenter)

        self.setLayout(self.layout)

        self.is_recording = False
        self.update_button_styles()

        # Create a QTimer to call rclpy.spin_once periodically
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(100)  # Spin at 10 Hz

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def start_recording(self):
        if not self.is_recording:
            self.node.start_recording()
            if self.node.current_status == 1:
                self.is_recording = True
                self.update_button_styles()

    def stop_recording(self):
        if self.is_recording:
            self.node.stop_recording()
            self.is_recording = False
            self.update_button_styles()

    def update_button_styles(self):
        if self.is_recording:
            self.start_button.setEnabled(False)
            self.start_button.setStyleSheet("background-color: green")
            self.stop_button.setEnabled(True)
            self.stop_button.setStyleSheet("background-color: none")
        else:
            self.start_button.setEnabled(True)
            self.start_button.setStyleSheet("background-color: none")
            self.stop_button.setEnabled(False)
            self.stop_button.setStyleSheet("background-color: red")

    def resizeEvent(self, event):
        # Calculate the size for the icons based on the maximum button size
        button_size = min(self.start_button.maximumWidth(), self.start_button.maximumHeight())
        icon_size = button_size - 20  # Adjust for padding

        self.start_button.setIconSize(QSize(icon_size, icon_size))
        self.stop_button.setIconSize(QSize(icon_size, icon_size))

        super().resizeEvent(event)

def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    topic_names = ['/zed_doorway/zed_node_doorway/left_raw/image_raw_color', '/zed_doorway/zed_node_doorway/body_trk/skeletons']  # Replace with your topic names
    main_window = MainWindow(None)
    node = BagRecorder(main_window.info_label, topic_names)
    main_window.node = node
    main_window.show()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        app.exec_()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
