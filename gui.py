#pip install rclpy PyQt5

import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
import subprocess

class BagRecorder(Node):
    def __init__(self, topic_name):
        super().__init__('bag_recorder')
        self.topic_name = topic_name
        self.process = None

    def start_recording(self):
        if self.process is None:
            self.process = subprocess.Popen(['ros2', 'bag', 'record', self.topic_name])
            self.get_logger().info(f"Started recording {self.topic_name}")

    def stop_recording(self):
        if self.process is not None:
            self.process.terminate()
            self.process.wait()
            self.process = None
            self.get_logger().info(f"Stopped recording {self.topic_name}")

class MainWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node

        self.setWindowTitle("ROS 2 Bag Recorder")
        self.setGeometry(100, 100, 300, 200)

        self.layout = QVBoxLayout()

        self.start_button = QPushButton("Start Recording")
        self.start_button.clicked.connect(self.node.start_recording)
        self.layout.addWidget(self.start_button)

        self.stop_button = QPushButton("Stop Recording")
        self.stop_button.clicked.connect(self.node.stop_recording)
        self.layout.addWidget(self.stop_button)

        self.setLayout(self.layout)

def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    fixed_topic_name = '/image/rgb'  # Set the fixed topic name here
    node = BagRecorder(fixed_topic_name)
    main_window = MainWindow(node)
    main_window.show()

    app.exec_()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
