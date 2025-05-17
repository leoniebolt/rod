import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QGridLayout
import rospy
from std_msgs.msg import String

class RobotHMI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        rospy.init_node('qt_hmi_node', anonymous=True)
        self.publisher = rospy.Publisher('/robot_control_topic', String, queue_size=10)

    def initUI(self):
        self.setWindowTitle('Robot Control HMI')
        layout = QGridLayout()

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
            btn.setFixedSize(60, 60)
            btn.clicked.connect(lambda _, cmd=self.buttons[label]: self.send_command(cmd))
            layout.addWidget(btn, pos[0], pos[1])

        self.setLayout(layout)
        self.resize(250, 300)
        self.show()

    def send_command(self, command):
        rospy.loginfo(f"Command sent: {command}")
        self.publisher.publish(command)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = RobotHMI()
    sys.exit(app.exec_())
