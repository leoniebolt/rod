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
        rospy.init_node(nodename, anonymous=False)

        # Warten bis move_group bereit ist
        self.wait_for_move_group()

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander(groupname)

        # Scaling Factors + Tolerances
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        self.move_group.set_max_velocity_scaling_factor(0.1)
        #self.move_group.set_goal_tolerance(0.5)
        #self.move_group.set_goal_position_tolerance(0.1)
        #self.move_group.set_goal_orientation_tolerance(0.5)

        # Initialize self variables
        self.groupname = groupname
        self.goals = []
        self.joint_goals = []
        self.pose_goals = []

    def wait_for_move_group(self):
        # warten bis der move_group ActionServer verfügbar ist
        print("Warte auf move_group Action Server...")
        client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
        if not client.wait_for_server(rospy.Duration(10)):
            print("move_group Action Server nicht erreichbar, warte länger...")
            while not client.wait_for_server(rospy.Duration(1)):
                if rospy.is_shutdown():
                    raise rospy.ROSInterruptException("ROS wurde beendet während gewartet wurde.")
                print(".", end="", flush=True)
        print("\nmove_group Action Server ist verfügbar!")

    # Funktion um aktuelle Pose zu bekommen
    def get_current_pose(self):
        pose = self.move_group.get_current_pose().pose
        print(f"[{self.groupname}] Aktuelle Pose:")
        print(f"  Position: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
        print(f"  Orientierung: x={pose.orientation.x:.3f}, y={pose.orientation.y:.3f}, "
              f"z={pose.orientation.z:.3f}, w={pose.orientation.w:.3f}")
        return pose

    # Funktion um zu gewünschter Pose zu fahren
    def move_to_pose(self, pose, label=""):
        print(f"[{self.groupname}] Bewege zu Pose: {label}")
        self.move_group.set_pose_target(pose)
        success = self.move_group.go(wait=True)
        if not success:
            print(f"[{self.groupname}] Bewegung zu {label} fehlgeschlagen!")
        else:
            print(f"[{self.groupname}] Erfolg!")
        self.move_group.stop()
        self.move_group.clear_pose_targets()


    # Funktion um Gelenkwerte zu setzen
    def set_joint_target(self, joint_values):
        self.joint_goals.append(joint_values)

    # Funktion um Roboter zu bewegen
    def move(self):
        self.move_group.clear_pose_targets()
        try:
            for g in self.goals:
                self.move_group.set_pose_target(g)
                self.move_group.go(wait=True)
            for joints in self.joint_goals:
                print(f"[{self.groupname}] Moving to joint target: {joints}")
                self.move_group.set_joint_value_target(joints)
                self.move_group.go(wait=True)
                self.move_group.stop()
        except:
            print("Targets not reachable")
        finally:
            print("Stopping")
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            self.goals = []
            self.joint_goals = []

            
if __name__ == "__main__":

        # Poses for SCARA robot
    s_poses = {
        # WERTE NICHT VERÄNDERN!!!!
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
            position=geometry_msgs.msg.Point(x=2.397, y=-1.085, z=1.233),
            orientation=geometry_msgs.msg.Quaternion(x=-0.964, y=-0.264, z=-0.000, w=0.000)
            # Joint (grad) [-90, -60, 0.009, 1]
            # Joint (rad) [-1.57, -1.044, 0.009, 0.018]
        ),
        "s_place": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=2.397, y=-1.085, z=1.137),
            orientation=geometry_msgs.msg.Quaternion(x=-0.964, y=-0.264, z=-0.000, w=0.000)
            # Joint (grad) [-90, -60, 0.091, 1]
            # Joint (rad) [-1.57, -1.044, 0.091, 0.018]
        )
    }

    # Poses for pillar
    pi_poses = {
        # DO NOT CHANGE VALUES
        "pillar_home":geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=4.545, y=-1.745, z=1.505),
            orientation=geometry_msgs.msg.Quaternion(x=0.707, y=-0.025, z=-0.025, w=0.707)
            # Joint (grad) = [0, 0, 0, 0, 0, 0]
            # Joint (grad) = [0, 0, 0, 0, 0, 0]
        ),
        "pi_above_pick_up":geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=-0.867, y=-1.121, z=1.591),
            orientation=geometry_msgs.msg.Quaternion(x=0.703, y=-0.001, z=0.712, w=0.000)
            # Joint (grad) = [0, 0, 0, 0, 0, 0]
            # Joint (grad) = [0, 0, 0, 0, 0, 0]
        ),
        "pi_pick_up":geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=-0.867, y=-1.121, z=1.591),
            orientation=geometry_msgs.msg.Quaternion(x=0.703, y=-0.001, z=0.712, w=0.000)
            # Joint (grad) = [0, 0, 0, 0, 0, 0]
            # Joint (grad) = [0, 0, 0, 0, 0, 0]
        ),
        "pi_above_place_up":geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=-0.867, y=-1.121, z=1.591),
            orientation=geometry_msgs.msg.Quaternion(x=0.703, y=-0.001, z=0.712, w=0.000)
            # Joint (grad) = [0, 0, 0, 0, 0, 0]
            # Joint (grad) = [0, 0, 0, 0, 0, 0]
        ),
        "pi_place_up":geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=-0.867, y=-1.121, z=1.591),
            orientation=geometry_msgs.msg.Quaternion(x=0.703, y=-0.001, z=0.712, w=0.000)
            # Joint (grad) = [0, 0, 0, 0, 0, 0]
            # Joint (grad) = [0, 0, 0, 0, 0, 0]
        ),
    }


    # Poses for SIXAXIS robot
    sa_poses = {
        # DO NOT CHANGE VALUES
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
            position=geometry_msgs.msg.Point(x=-2.425, y=-1.040, z=1.067),
            orientation=geometry_msgs.msg.Quaternion(x=-0.715, y=-0.699, z=0.004, w=0.012)
            # Joint (grad) = [173, -4, -27, 0, -66, -7]
            # Joint (rad) = [3.0194, -0.0698, -0.4712, 0.0, -1.1519, -0.1222]
        ),
        "sa_place": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=-2.771, y=-1.029, z=0.421),
            orientation=geometry_msgs.msg.Quaternion(x=-0.717, y=-0.697, z=0.007, w=0.018)
            # Joint (grad) = [180, 55, -12, 0, -23, -5]
            # Joint (rad) = [3.129, 0.963, -0.204, 0.007, -0.400, -0.085]
        )
    }

    # initialize robot instances
    scara = DemoRobot(groupname="scara")    
    sixaxis = DemoRobot(groupname="sixaxis")
    pillar = DemoRobot(groupname="pillar")


    # testing
    print("scara pose:\n")
    scara.get_current_pose()
    print("\n. \n. \n. \n")

    print("pillar pose:\n")
    pillar.get_current_pose()
    print("\n. \n. \n. \n")

    print("sixaxis pose:\n")
    sixaxis.get_current_pose()
    print("\n. \n. \n. \n")


    scara.move_to_pose(s_poses["scara_home"], "scara home")
    pillar.move_to_pose(pi_poses["pillar_home"], "Pillar home")
    sixaxis.move_to_pose(sa_poses["sixaxis_home"], "sixaxis home")

"""
    try:
        while True:
            print("\n--- Starting new cycle ---\n")

            scara_mover = DemoRobot(groupname="scara")
            pillar_mover = DemoRobot(groupname="pillar")
            sixaxis_mover = DemoRobot(groupname="sixaxis")

            # Beispielpositionen anfahren
            scara_mover.move_to_pose(s_poses["scara_home"], "Scara home\n")
            #scara_mover.move_to_pose(s_poses["scara_home"], "Scara home\n")
            #scara_mover.move_to_pose(s_poses["scara_home"], "Scara home\n")
            #scara_mover.move_to_pose(s_poses["scara_home"], "Scara home\n")
            #scara.move_to_pose(s_poses["s_above_pick_up"], "scara above pick up")

            pillar.move_to_pose(pi_poses["pillar_home"], "pillar home")


            sixaxis.get_current_pose()


            # SCARA movement
            scara.get_current_pose()
            print("\n. \n. \n")
            scara.set_joint_target([0, 0, 0, 0])         
            scara.move()

            scara.get_current_pose()
            print("\n. \n. \n")
            scara.set_joint_target([1.572, 1.036, 0, 0.0])         
            scara.move()

            print("\n. \n. \n")
            scara.set_joint_target([1.572, 1.036, 0.091, 0.0])        
            scara.move()

            print("\n. \n. \n")
            scara.set_joint_target([1.572, 1.036, 0, 0.0])         
            scara.move()

            print("\n. \n. \n")
            scara.set_joint_target([-1.57, -1.044, 0.009, 0.018])     
            scara.move()

            print("\n. \n. \n")
            scara.set_joint_target([-1.57, -1.044, 0.091, 0.018])         
            scara.move()

            print("\n. \n. \n")
            scara.set_joint_target([-1.57, -1.044, 0.009, 0.018])     
            scara.move()

            scara.get_current_pose()
            print("\n. \n. \n")
            scara.set_joint_target([0, 0, 0, 0])         
            scara.move()

            # SIXAXIS movement
            print("\n. \n. \n")
            sixaxis.set_joint_target([0.0873, 0.6632, 0.9948, 0.0, -1.850, 0.0698])         
            sixaxis.move()

            print("\n. \n. \n")
            sixaxis.set_joint_target([0.0524, 0.6283, 0.4014, 0.0, -1.3439, 0.0524])        
            sixaxis.move()

            print("\n. \n. \n")
            sixaxis.set_joint_target([0.0873, 0.6632, 0.9948, 0.0, -1.850, 0.0698])         
            sixaxis.move()

            print("\n. \n. \n")
            sixaxis.set_joint_target([3.0194, -0.0698, -0.4712, 0.0, -1.1519, -0.1222])     
            sixaxis.move()

            print("\n. \n. \n")
            sixaxis.set_joint_target([3.129, 0.963, -0.204, 0.007, -0.400, -0.085])         
            sixaxis.move()

            print("\n. \n. \n")
            sixaxis.set_joint_target([3.0194, -0.0698, -0.4712, 0.0, -1.1519, -0.1222])     
            sixaxis.move()

            print("\n. \n. \n")
            sixaxis.set_joint_target([0, 0, 0, 0, 0, 0])                                    
            sixaxis.move()

            print("\n--- Cycle complete ---\n")

            #time.sleep(1)  # Optional pause between cycles

    except KeyboardInterrupt:
        print("\n\nStopped by user with Ctrl+C. Exiting loop.\n")"""
