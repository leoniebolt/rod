#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QGridLayout
import rospy
from std_msgs.msg import String
<<<<<<< HEAD
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QGridLayout, QComboBox, QVBoxLayout, QLabel
=======
>>>>>>> aedb43ab837998f1856a685f83b851b29b963343

class RobotHMI(QWidget):
    def __init__(self):
        super().__init__()
<<<<<<< HEAD
        rospy.init_node('qt_hmi_node', anonymous=True)
        self.current_robot = 'scara'

=======
>>>>>>> aedb43ab837998f1856a685f83b851b29b963343
        self.initUI()
        rospy.init_node('qt_hmi_node', anonymous=True)
        self.publisher = rospy.Publisher('/robot_control_topic', String, queue_size=10)

    def initUI(self):
<<<<<<< HEAD
        self.setWindowTitle('Roboter HMI')

        layout = QVBoxLayout()
        self.setLayout(layout)

        self.robot_selector = QComboBox()
        self.robot_selector.addItems(['scara', 'ur'])
        self.robot_selector.currentTextChanged.connect(self.change_robot)

        layout.addWidget(QLabel("Zielroboter:"))
        layout.addWidget(self.robot_selector)

        grid = QGridLayout()
=======
        self.setWindowTitle('Robot Control HMI')
        layout = QGridLayout()
>>>>>>> aedb43ab837998f1856a685f83b851b29b963343

        self.buttons = {
            '↑': 'up', '↓': 'down', '←': 'left', '→': 'right',
            '⟲': 'rotate_ccw', '⟳': 'rotate_cw',
            'Start': 'start', 'Stop': 'stop'
        }

        positions = {
            '↑': (0, 1), '←': (1, 0), '→': (1, 2), '↓': (2, 1),
            '⟲': (3, 0), '⟳': (3, 2), 'Start': (4, 0), 'Stop': (4, 2)
        }

        for label, pos in positions.items():
            btn = QPushButton(label)
<<<<<<< HEAD
=======
            btn.setFixedSize(60, 60)
>>>>>>> aedb43ab837998f1856a685f83b851b29b963343
            btn.clicked.connect(lambda _, cmd=self.buttons[label]: self.send_command(cmd))
            layout.addWidget(btn, pos[0], pos[1])

        self.setLayout(layout)
        self.resize(250, 300)
        self.show()

<<<<<<< HEAD
    def change_robot(self, robot):
        self.current_robot = robot

    def send_command(self, command):
        topic = f"/{self.current_robot}_control_topic"
        pub = rospy.Publisher(topic, String, queue_size=10)
        rospy.sleep(0.1)
        pub.publish(command)
        rospy.loginfo(f"Befehl '{command}' an {topic} gesendet.")
=======
    def send_command(self, command):
        rospy.loginfo(f"Command sent: {command}")
        self.publisher.publish(command)
>>>>>>> aedb43ab837998f1856a685f83b851b29b963343

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = RobotHMI()
    sys.exit(app.exec_())
