#!/usr/bin/env python3

## @file robot_hmi.py
#  @brief Qt-based GUI for controlling a robot using ROS.
#  @author Eeman, Felix, Leonie
#  @date 2025-06-27

import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QGridLayout,
    QComboBox, QVBoxLayout, QLabel
)
import rospy
from std_msgs.msg import String

## @class RobotHMI
#  @brief Eine einfache grafische Oberfläche zur Robotersteuerung

class RobotHMI(QWidget):
    ## Constructor: Initializes the ROS node and GUI.
    def __init__(self):
        super().__init__()
        rospy.init_node('qt_hmi_node', anonymous=True)
        self.current_robot = 'sixaxis'
        self.publisher = rospy.Publisher('/robot_control_topic', String, queue_size=10)
        self.initUI()

     ## Initializes the Qt GUI layout and buttons.
    def initUI(self):
        self.setWindowTitle('Roboter HMI')

        # Main layout
        layout = QVBoxLayout()
        self.setLayout(layout)

        layout.addWidget(QLabel("Zielroboter: Sixaxis"))

        # Grid layout for directional and control buttons
        grid = QGridLayout()

        ## Button labels mapped to commands
        self.buttons = {
            '↑': 'up', '↓': 'down', '←': 'left', '→': 'right',
            'Start': 'start', 'Stop': 'stop'
        }

        ## Grid positions for each button
        positions = {
            '↑': (0, 1), '←': (1, 0), '→': (1, 2), '↓': (2, 1),
            'Start': (4, 0), 'Stop': (4, 2)
        }

        for label, pos in positions.items():
            btn = QPushButton(label)
            btn.clicked.connect(lambda _, cmd=self.buttons[label]: self.send_command(cmd))
            grid.addWidget(btn, pos[0], pos[1])

        layout.addLayout(grid)

        self.resize(250, 300)
        self.show()

    ## Changes the currently selected robot.
    #  @param robot The name of the robot to control.
    def change_robot(self, robot):
        self.current_robot = robot

    ## Publishes a control command to the appropriate ROS topic.
    #  @param command The command string to send (e.g., 'up', 'start').
    def send_command(self, command):
        topic = f"/{self.current_robot}_control_topic"
        pub = rospy.Publisher(topic, String, queue_size=10)
        rospy.sleep(0.1)
        pub.publish(command)
        rospy.loginfo(f"Befehl '{command}' an {topic} gesendet.")

## Main entry point for the Qt application.
if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = RobotHMI()
    sys.exit(app.exec_())
