#!/usr/bin/env python3
import sys
import rospy
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QGridLayout, QComboBox, QVBoxLayout, QLabel
from PyQt5.QtCore import QTimer
from std_msgs.msg import String

class RobotHMI(QWidget):
    def __init__(self):
        super().__init__()

        rospy.init_node('qt_hmi_node', anonymous=True)

        self.publishers = {
            "scara": rospy.Publisher("/scara_control_topic", String, queue_size=10),
            "ur": rospy.Publisher("/ur_control_topic", String, queue_size=10)
        }

        self.current_robot = "scara"
        self.initUI()

    def initUI(self):
        self.setWindowTitle("Roboter HMI")

        main_layout = QVBoxLayout()
        self.setLayout(main_layout)

        # Dropdown zur Roboterwahl
        self.robot_selector = QComboBox()
        self.robot_selector.addItems(["scara", "ur"])
        self.robot_selector.currentTextChanged.connect(self.change_robot)

        main_layout.addWidget(QLabel("Zielroboter:"))
        main_layout.addWidget(self.robot_selector)

        # Button-Layout
        grid = QGridLayout()
        main_layout.addLayout(grid)

        self.buttons = {
            "↑": "up", "↓": "down", "←": "left", "→": "right",
            "⟲": "rotate_ccw", "⟳": "rotate_cw",
            "Start": "start", "Stop": "stop"
        }

        positions = {
            "↑": (0, 1), "←": (1, 0), "→": (1, 2), "↓": (2, 1),
            "⟲": (3, 0), "⟳": (3, 2), "Start": (4, 0), "Stop": (4, 2)
        }

        for label, pos in positions.items():
            btn = QPushButton(label)
            btn.setFixedSize(60, 60)
            btn.clicked.connect(lambda _, cmd=self.buttons[label]: self.send_command(cmd))
            grid.addWidget(btn, pos[0], pos[1])

        self.resize(250, 300)
        self.show()

    def change_robot(self, robot):
        self.current_robot = robot
        rospy.loginfo(f"[HMI] Zielroboter geändert: {robot}")

    def send_command(self, command):
        if self.current_robot in self.publishers:
            self.publishers[self.current_robot].publish(String(data=command))
            rospy.loginfo(f"[HMI] Befehl '{command}' an {self.current_robot}_control_topic gesendet.")
        else:
            rospy.logwarn(f"[HMI] Kein Publisher für Roboter '{self.current_robot}' gefunden.")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = RobotHMI()
    sys.exit(app.exec_())
