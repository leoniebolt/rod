import sys
import rospy
from std_msgs.msg import String
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QGridLayout, QComboBox, QVBoxLayout, QLabel
)

class RobotHMI(QWidget):
    def __init__(self):
        super().__init__()
        rospy.init_node('qt_hmi_node', anonymous=True)
        self.current_robot = 'scara'  # Default

        self.initUI()

    def initUI(self):
        self.setWindowTitle('Dual Robot Control HMI')

        layout = QVBoxLayout()
        self.setLayout(layout)

        # Robot selection dropdown
        self.robot_selector = QComboBox()
        self.robot_selector.addItems(['scara', 'ur5'])
        self.robot_selector.currentTextChanged.connect(self.change_robot)

        layout.addWidget(QLabel("Roboter auswählen:"))
        layout.addWidget(self.robot_selector)

        # Control button grid
        grid = QGridLayout()

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
            btn.setFixedSize(70, 50)
            if label == 'Start':
                btn.setStyleSheet("background-color: lightgreen")
            elif label == 'Stop':
                btn.setStyleSheet("background-color: lightcoral")
            btn.clicked.connect(lambda _, cmd=self.buttons[label]: self.send_command(cmd))
            grid.addWidget(btn, pos[0], pos[1])

        layout.addLayout(grid)
        self.resize(300, 350)
        self.show()

    def change_robot(self, robot_name):
        self.current_robot = robot_name
        rospy.loginfo(f"Zielroboter gesetzt auf: {robot_name}")

    def send_command(self, command):
        topic = f"/{self.current_robot}_control_topic"
        pub = rospy.Publisher(topic, String, queue_size=10)
        rospy.sleep(0.1)  # Kurze Pause, damit Publisher bereit ist
        pub.publish(command)
        rospy.loginfo(f"Befehl '{command}' an {topic} gesendet.")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = RobotHMI()
    sys.exit(app.exec_())
