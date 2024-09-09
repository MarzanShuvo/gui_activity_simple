#!/usr/bin/env python3
# pip install rclpy PyQt5
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int64, String
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QSizePolicy, QSpacerItem, QLineEdit, QMessageBox
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import QSize, Qt, QTimer

class BagRecorder(Node):
    def __init__(self, info_label, topic_names):
        super().__init__('bag_recorder')
        self.info_label = info_label
        self.publisher_status = self.create_publisher(Int64, 'recording_status', 10)
        self.publisher_name = self.create_publisher(String, 'recording_name', 10)
        self.current_status = 0
        self.timer = self.create_timer(1.0, self.publish_data)
        self.topic_names = topic_names
        self.name_to_publish = ""
        self.used_names = []  # List to store used names

    def set_name(self, name):
        self.name_to_publish = name
        self.used_names.append(name)  # Store the name in the list of used names

    def start_recording(self):
        if self.topics_available():
            self.current_status = 1
            self.info_label.setText(f"Started recording with name: {self.name_to_publish}")
            self.get_logger().info("Started recording")
        else:
            self.info_label.setText("One or more topics not available")
            self.get_logger().info("One or more topics not available")

    def stop_recording(self):
        self.current_status = 0
        self.info_label.setText("Stopped recording")
        self.get_logger().info("Stopped recording")

    def publish_data(self):
        msg_status = Int64()
        msg_status.data = self.current_status
        self.publisher_status.publish(msg_status)
        self.get_logger().info(f"Published status: {self.current_status}")

        if self.current_status == 1 and self.name_to_publish:
            msg_name = String()
            msg_name.data = self.name_to_publish
            self.publisher_name.publish(msg_name)
            self.get_logger().info(f"Published name: {self.name_to_publish}")
        elif self.current_status == 0:
            self.get_logger().info("Recording is inactive, name will not be published")

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

        self.name_input = QLineEdit(self)
        self.name_input.setPlaceholderText("Enter recording name")
        self.layout.addWidget(self.name_input)

        self.start_button = QPushButton("")
        self.start_button.setIcon(QIcon('start_button.png'))
        self.start_button.clicked.connect(self.start_recording)
        self.start_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.start_button.setMaximumSize(200, 200)
        self.button_layout.addWidget(self.start_button)

        self.stop_button = QPushButton("")
        self.stop_button.setIcon(QIcon('stop_button.png'))
        self.stop_button.clicked.connect(self.stop_recording)
        self.stop_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.stop_button.setMaximumSize(200, 200)
        self.button_layout.addWidget(self.stop_button)

        self.layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        self.layout.addLayout(self.button_layout)
        self.layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))

        self.info_label = QLabel("Ready")
        self.layout.addWidget(self.info_label, alignment=Qt.AlignCenter)

        self.setLayout(self.layout)

        self.is_recording = False

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(100)

        self.start_timer = QTimer(self)
        self.start_timer.setSingleShot(True)
        self.start_timer.timeout.connect(self.enable_start_button)

        self.recording_timer = QTimer(self)
        self.recording_timer.setSingleShot(True)
        self.recording_timer.timeout.connect(self.auto_stop_recording)

        self.update_button_styles()

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def start_recording(self):
        name = self.name_input.text()

        # Check if the name is unique
        if not name:
            self.info_label.setText("Please enter a name before starting")
            return
        elif name in self.node.used_names:
            QMessageBox.warning(self, "Duplicate Name", "This name has already been used. Please enter a unique name.")
            return

        self.node.set_name(name)

        if not self.is_recording:
            self.node.start_recording()
            if self.node.current_status == 1:
                self.is_recording = True
                self.update_button_styles()
                self.recording_timer.start(10*60*1000)

    def stop_recording(self):
        if self.is_recording:
            self.node.stop_recording()
            self.is_recording = False
            self.start_button.setEnabled(False)
            self.start_timer.start(30*1000)
            self.recording_timer.stop()
            self.update_button_styles()

    def auto_stop_recording(self):
        self.stop_recording()
        self.info_label.setText("Recording stopped automatically after 10 minutes")

    def enable_start_button(self):
        self.start_button.setEnabled(True)
        self.update_button_styles()

    def update_button_styles(self):
        if self.is_recording:
            self.start_button.setEnabled(False)
            self.start_button.setStyleSheet("background-color: green")
            self.stop_button.setEnabled(True)
            self.stop_button.setStyleSheet("background-color: none")
        else:
            self.start_button.setEnabled(True if not self.start_timer.isActive() else False)
            self.start_button.setStyleSheet("background-color: none")
            self.stop_button.setEnabled(False)
            self.stop_button.setStyleSheet("background-color: red")

    def resizeEvent(self, event):
        button_size = min(self.start_button.maximumWidth(), self.start_button.maximumHeight())
        icon_size = button_size - 20

        self.start_button.setIconSize(QSize(icon_size, icon_size))
        self.stop_button.setIconSize(QSize(icon_size, icon_size))

        super().resizeEvent(event)

def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    topic_names = [
        '/zed_doorway/zed_node_doorway/left/image_rect_color',
        '/zed_doorway/zed_node_doorway/body_trk/skeletons',
        '/zed_kitchen/zed_node_kitchen/left/image_rect_color',
        '/zed_kitchen/zed_node_kitchen/body_trk/skeletons'
    ]
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
