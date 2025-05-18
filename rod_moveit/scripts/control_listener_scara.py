#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# SCARA hat diese Joints laut deinem Param: sj1, sj2, sj3, sjEE
JOINT_NAMES = ['sj1', 'sj2', 'sj3', 'sjEE']

def callback(msg):
    command = msg.data
    traj = JointTrajectory()
    traj.joint_names = JOINT_NAMES

    point = JointTrajectoryPoint()
    point.time_from_start = rospy.Duration(1.0)  # 1 Sekunde für die Bewegung

    if command == "up":
        point.positions = [0.2, 0.0, 0.0, 0.0]
    elif command == "down":
        point.positions = [-0.2, 0.0, 0.0, 0.0]
    elif command == "rotate_ccw":
        point.positions = [0.0, 0.0, 0.0, 0.5]
    elif command == "rotate_cw":
        point.positions = [0.0, 0.0, 0.0, -0.5]
    elif command == "stop":
        point.positions = [0.0, 0.0, 0.0, 0.0]
    else:
        rospy.logwarn(f"Unbekannter Befehl: {command}")
        return

    traj.points.append(point)
    pub.publish(traj)
    rospy.loginfo(f"[SCARA] Befehl '{command}' gesendet → {point.positions}")

rospy.init_node('control_listener_scara')
pub = rospy.Publisher('/scara_controller/command', JointTrajectory, queue_size=10)
rospy.Subscriber('/scara_control_topic', String, callback)
rospy.spin()
