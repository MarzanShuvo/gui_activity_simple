#!/usr/bin/env python3
#pip install rclpy PyQt5
import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QSizePolicy, QSpacerItem
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import QSize, Qt
import subprocess

class BagRecorder(Node):
    def __init__(self, topic_name, info_label):
        super().__init__('bag_recorder')
        self.topic_name = topic_name
        self.process = None
        self.info_label = info_label

    def start_recording(self):
        if self.process is None:
            self.process = subprocess.Popen(['ros2', 'bag', 'record', self.topic_name])
            self.get_logger().info(f"Started recording {self.topic_name}")
            self.info_label.setText(f"Started recording {self.topic_name}")

    def stop_recording(self):
        if self.process is not None:
            self.process.terminate()
            self.process.wait()
            self.process = None
            self.get_logger().info(f"Stopped recording {self.topic_name}")
            self.info_label.setText(f"Stopped recording {self.topic_name}")

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

    def start_recording(self):
        if not self.is_recording:
            self.node.start_recording()
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
    fixed_topic_name = '/image/rgb'  # Set the fixed topic name here
    main_window = MainWindow(None)
    node = BagRecorder(fixed_topic_name, main_window.info_label)
    main_window.node = node
    main_window.show()

    app.exec_()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
