#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np
from math import pi
import time
import actionlib
from moveit_msgs.msg import MoveGroupAction


class DemoRobot:
    def __init__(self, nodename="rod_node", groupname="rod_group"):
        # Initializing ROS node
        rospy.init_node(nodename, anonymous=False)

        # Waiting for move_group to be ready
        self.wait_for_move_group()

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander(groupname)

        # Scaling Factors + Tolerances
        self.move_group.set_max_acceleration_scaling_factor(1.0)
        self.move_group.set_max_velocity_scaling_factor(1.0)

        # Initialize self variables
        self.groupname = groupname
        self.joint_goals = []

    def wait_for_move_group(self):
        # Waiting for move_group ActionServer to be available
        print("Waiting for move_group Action Server...")
        client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
        if not client.wait_for_server(rospy.Duration(10)):
            print("move_group Action Server not reachable, waiting longer...")
            while not client.wait_for_server(rospy.Duration(1)):
                if rospy.is_shutdown():
                    raise rospy.ROSInterruptException("ROS was ended while starting.")
                print(".", end="", flush=True)
        print("\nmove_group Action Server is available!")

    # Function to print current end-effector pose of robot group
    def get_current_pose(self):
        pose = self.move_group.get_current_pose().pose
        print(f"[{self.groupname}] Current Pose:")
        print(f"  Position: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
        print(f"  Orientation: x={pose.orientation.x:.3f}, y={pose.orientation.y:.3f}, "
              f"z={pose.orientation.z:.3f}, w={pose.orientation.w:.3f}")
        return pose

    # Function to set joint values of robot
    def set_joint_target(self, joint_values):
        self.joint_goals.append(joint_values)

    # Execute all goals in self.goals (pose goals) and self.joint_goals (joint goals)
    def move(self):
            self.move_group.clear_pose_targets()    # Clear any previous pose targets
            try:
                # Execute all joint goals
                for joints in self.joint_goals:
                    print(f"[{self.groupname}] Moving to joint target: {joints}")
                    self.move_group.set_joint_value_target(joints)
                    self.move_group.go(wait=True)
                    self.move_group.stop()
            except:
                print("Targets not reachable")
            finally:
                # Clean up: stop movement, clear targets and reset goal lists
                print("Stopping")
                self.move_group.stop()
                self.joint_goals = []
            
if __name__ == "__main__":

    # Poses for SCARA
    s_poses = {
        "scara_home": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=7.012, y=-1.134, z=1.224),
            orientation=geometry_msgs.msg.Quaternion(x=0.998, y=-0.064, z=0.000, w=0.000)
            # Joint (grad) = [0, 0, 0.009, 1]
            # Joint (rad) = [0, 0, 0, 0]
        ),

        "s_above_pick_up": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=2.465, y=-0.200, z=1.239),
            orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=-0.000, w=0.000)
            # Joint (grad) = [90, 60, 0.009, 1]
            # Joint (rad) = [1.572, 1.036, 0, 0.0]
        ),
        "s_pick_up": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=2.465, y=-0.200, z=1.137),
            orientation=geometry_msgs.msg.Quaternion(x=-0.964, y=-0.264, z=-0.000, w=0.000)
            # Joint (grad) = [90, 60, 0.091, 1]
            # Joint (rad) = [1.572, 1.036, 0.091, 0.0]
        ),
        "s_above_place": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=6.514, y=-1.559, z=1.224),
            orientation=geometry_msgs.msg.Quaternion(x=-0.999, y=-0.044, z=0.000, w=0.000)
            # Joint (grad) [-114, -79, 0, 313]
            # Joint (rad) [-1.992, -1.381, 0, 5.456]
        ),
        "s_place": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=2.397, y=-1.085, z=1.137),
            orientation=geometry_msgs.msg.Quaternion(x=-0.964, y=-0.264, z=-0.000, w=0.000)
            # Joint (grad) [-114, -79, 0.91, 313]
            # Joint (rad) [-1.992, -1.381, 0.91, 5.456]
        )
    }

    # Poses for Pillar
    pi_poses = {
        "pillar_home":geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=4.545, y=-1.745, z=1.505),
            orientation=geometry_msgs.msg.Quaternion(x=0.707, y=-0.025, z=-0.025, w=0.707)
            # Joint (grad) = [0, 0, 0, 0]
            # Joint (rad) = [0, 0, 0, 0]
        ),
        "pi_above_pick_up":geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=5.430, y=-2.009, z=1.333),
            orientation=geometry_msgs.msg.Quaternion(x=0.497, y=-0.503, z=-0.503, w=0.497)
            # Joint (grad) = [-0.213, -132, 25, 20]
            # Joint (rad) = [-0.213, -2.299, 0.434, 0.353]
        ),
        "pi_pick_up":geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=5.430, y=-2.009, z=1.072),
            orientation=geometry_msgs.msg.Quaternion(x=0.497, y=-0.503, z=-0.503, w=0.497)
            # Joint (grad) = [-0.417, -132, 25, 20]
            # Joint (rad) = [-0.417, -2.299, 0.434, 0.353]
        ),
        "pi_above_place_up":geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=4.855, y=-1.524, z=1.353),
            orientation=geometry_msgs.msg.Quaternion(x=0.707, y=0.014, z=0.014, w=0.707)
            # Joint (grad) = [-0.193, -102, 135, -27]
            # Joint (rad) = [-0.193, -1.783, 2.359, -0.467]
        ),
        "pi_place_up":geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=4.855, y=-1.524, z=1.204),
            orientation=geometry_msgs.msg.Quaternion(x=0.707, y=0.014, z=0.014, w=0.707)
            # Joint (grad) = [-0.342, -102, 135, -27]
            # Joint (rad) = [-0.342, -1.783, 2.359, -0.467]
        ),
    }


    # Poses for SIXAXIS
    sa_poses = {
        "sixaxis_home": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=2.960, y=-1.567, z=1.542),
            orientation=geometry_msgs.msg.Quaternion(x=0.500, y=0.509, z=0.491, w=0.500)
            # Joint (grad) = [0, 0, 0, 0, 0, 0]
            # Joint (grad) = [0, 0, 0, 0, 0, 0]
        ),
        "sa_above_pick_up": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=-0.688, y=-1.042, z=1.519),
            orientation=geometry_msgs.msg.Quaternion(x=1.000, y=-0.000, z=0.030, w=0.001)
            # Joint (grad) = [5, 38, 57, 0, -106, 4]
            # Joint (rad) = [0.0873, 0.6632, 0.9948, 0.0, -1.850, 0.0698]
        ),
        "sa_pick_up": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=-0.858, y=-1.087, z=1.198),
            orientation=geometry_msgs.msg.Quaternion(x=-1.000, y=-0.012, z=-0.022, w=0.005)
            # Joint (grad) = [3, 36, 23, 0, -77, 3]
            # Joint (rad) = [0.0524, 0.6283, 0.4014, 0.0, -1.3439, 0.0524]
        ),
        "sa_above_place": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=1.256, y=-1.563, z=0.536),
            orientation=geometry_msgs.msg.Quaternion(x=0.001, y=1.000, z=0.002, w=0.010)
            # Joint (grad) = [180, 60, -5, 0, -27, 0]
            # Joint (rad) = [3.134, 1.051, -0.085, -0.008, -0.472, -0.001]
        ),
        "sa_place": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=-2.771, y=-1.029, z=0.421),
            orientation=geometry_msgs.msg.Quaternion(x=-0.717, y=-0.697, z=0.007, w=0.018)
            # Joint (grad) = [180, 55, -12, 0, -23, -5]
            # Joint (rad) = [3.129, 0.963, -0.204, 0.007, -0.400, -0.085]
        )
    }

    # Initializing robot instances
    scara = DemoRobot(groupname="scara_group")    
    sixaxis = DemoRobot(groupname="sixaxis_group")
    pillar = DemoRobot(groupname="pillar_group")

    try:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
           # SCARA
           # above pick up
           scara.set_joint_target([1.572, 1.036, 0, 0.0])
           scara.move()
           if rospy.is_shutdown():
                break
           # pick up
           scara.set_joint_target([1.572, 1.036, 0.091, 0.0])
           scara.move()
           if rospy.is_shutdown():
                break
           # above pick up
           scara.set_joint_target([1.572, 1.036, 0, 0.0])
           scara.move()
           if rospy.is_shutdown():
                break
           # above place
           scara.set_joint_target([-1.9, -1.044, 0.009, 0.018])
           scara.move()
           if rospy.is_shutdown():
                break
           # place
           scara.set_joint_target([-1.9, -1.044, 0.091, 0.018])
           scara.move()
           if rospy.is_shutdown():
                break
           # above place
           scara.set_joint_target([-1.9, -1.044, 0.009, 0.018])
           scara.move()
           if rospy.is_shutdown():
                break
           # home
           scara.set_joint_target([0, 0, 0, 0.0])
           scara.move()
           if rospy.is_shutdown():
                break
        # PILLER
           # above pick
           pillar.set_joint_target([-0.213, -2.299, 0.434, 0.353])
           pillar.move()
           if rospy.is_shutdown():
                break
           # pick
           pillar.set_joint_target([-0.417, -2.299, 0.434, 0.353])
           pillar.move()
           if rospy.is_shutdown():
                break
           # above pick
           pillar.set_joint_target([-0.213, -2.299, 0.434, 0.353])
           pillar.move()
           if rospy.is_shutdown():
                break
           # above place
           pillar.set_joint_target([-0.193, -1.783, 2.359, -0.467])
           pillar.move()
           if rospy.is_shutdown():
                break
           # place
           pillar.set_joint_target([-0.342, -1.783, 2.359, -0.467])
           pillar.move()
           if rospy.is_shutdown():
                break
           # above place
           pillar.set_joint_target([-0.193, -1.783, 2.359, -0.467])
           pillar.move()
           if rospy.is_shutdown():
                break
           # home
           pillar.set_joint_target([0, 0, 0, 0])
           pillar.move()
           if rospy.is_shutdown():
                break
        # SIXAXIS
           # above pick
           sixaxis.set_joint_target([0.0873, 0.6632, 0.9948, 0.0, -1.850, 0.0698])
           sixaxis.move()
           if rospy.is_shutdown():
                break
           # pick
           sixaxis.set_joint_target([0.0524, 0.6283, 0.4014, 0.0, -1.3439, 0.0524])
           sixaxis.move()
           if rospy.is_shutdown():
                break
           # above pick
           sixaxis.set_joint_target([0.0873, 0.6632, 0.9948, 0.0, -1.850, 0.0698])
           sixaxis.move()
           if rospy.is_shutdown():
                break
           # above place
           sixaxis.set_joint_target([3.0194, -0.0698, -0.4712, 0.0, -1.1519, -0.1222])
           sixaxis.move()
           if rospy.is_shutdown():
                break
           # place
           sixaxis.set_joint_target([3.129, 0.963, -0.204, 0.007, -0.400, -0.085])
           sixaxis.move()
           if rospy.is_shutdown():
                break
           # above place
           sixaxis.set_joint_target([3.0194, -0.0698, -0.4712, 0.0, -1.1519, -0.1222])
           sixaxis.move()
           if rospy.is_shutdown():
                break
           # home
           sixaxis.set_joint_target([0, 0, 0, 0, 0, 0])
           sixaxis.move()
           if rospy.is_shutdown():
                break
    
    # Stopping via terminal with CTRL+C
    except KeyboardInterrupt:
        print("\nStopping detected! Stopping all robots...")
        scara.move_group.stop()
        pillar.move_group.stop()
        sixaxis.move_group.stop()
        print("Robots stopped. Programme ended.")

    # Cleaning up environment (stopping move_group + shutting down moveit_commander)
    finally:
        print("Clean and end.")
        scara.move_group.stop()
        pillar.move_group.stop()
        sixaxis.move_group.stop()
        moveit_commander.roscpp_shutdown()