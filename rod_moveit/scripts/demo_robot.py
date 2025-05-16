#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def send_trajectory(controller_name, joint_names, positions_list, duration=2.0):
    client = actionlib.SimpleActionClient(f'/{controller_name}/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joint_names

    for positions in positions_list:
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points.append(point)

    client.send_goal(goal)
    client.wait_for_result()

if __name__ == '__main__':
    rospy.init_node('dual_robot_trajectory')

    # Definieren Sie die Gelenknamen entsprechend Ihrer URDF
    sixaxis_joints = ['saj1', 'saj2', 'saj3', 'saj4', 'saj5', 'saj6']
    scara_joints = ['sj1', 'sj2', 'sj3', 'sjEE']

    # Beispielhafte Positionen für beide Roboter
    sixaxis_positions = [
        [0.0, -0.5, 0.5, 0.0, 0.5, 0.0],
        [0.2, -0.3, 0.6, 0.1, 0.4, 0.1]
    ]

    scara_positions = [
        [0.0, 0.5, 0.0, 0.0],
        [0.1, 0.4, 0.1, 0.1]
    ]

    rate = rospy.Rate(0.5)  # 0.5 Hz, also alle 2 Sekunden

    while not rospy.is_shutdown():
        send_trajectory('sixaxis_controller', sixaxis_joints, sixaxis_positions)
        send_trajectory('scara_controller', scara_joints, scara_positions)
        rate.sleep()
