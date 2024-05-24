import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QLineEdit

class BagRecorder(Node):
    def __init__(self, topic_name):
        super().__init__('bag_recorder')
        self.topic_name = topic_name
        self.process = None

    def start_recording(self):
        import subprocess
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

        self.label = QLabel("Enter Topic Name:")
        self.layout.addWidget(self.label)

        self.topic_input = QLineEdit()
        self.layout.addWidget(self.topic_input)

        self.start_button = QPushButton("Start Recording")
        self.start_button.clicked.connect(self.start_recording)
        self.layout.addWidget(self.start_button)

        self.stop_button = QPushButton("Stop Recording")
        self.stop_button.clicked.connect(self.stop_recording)
        self.layout.addWidget(self.stop_button)

        self.setLayout(self.layout)

    def start_recording(self):
        topic_name = self.topic_input.text()
        if topic_name:
            self.node.topic_name = topic_name
            self.node.start_recording()

    def stop_recording(self):
        self.node.stop_recording()

def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    node = BagRecorder('')
    main_window = MainWindow(node)
    main_window.show()

    app.exec_()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
