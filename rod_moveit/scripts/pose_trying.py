#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np
from math import pi

class DemoRobot:
    
    
    def __init__(self, nodename="DemoRobot", groupname="robotGroup"):
        rospy.init_node(nodename, anonymous=False)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander(groupname)
        
        #Acceleration and Speed factors
        self.move_group.set_max_acceleration_scaling_factor(1)
        self.move_group.set_max_velocity_scaling_factor(1)
        
        self.move_group.set_goal_tolerance(0.1)
        self.move_group.set_goal_position_tolerance(0.01)
        self.move_group.set_goal_orientation_tolerance(0.05)
        self.goals = []
        
        self.groupname = groupname
        self.joint_goals = []
        #self.pose_goals = []



    def get_current_pose(self):
        pose = self.move_group.get_current_pose().pose
        print(f"[{self.groupname}] Aktuelle Pose:")
        print(f"  Position: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
        print(f"  Orientierung: x={pose.orientation.x:.3f}, y={pose.orientation.y:.3f}, "
              f"z={pose.orientation.z:.3f}, w={pose.orientation.w:.3f}")
        return pose


    def move_to_pose(self, pose, label=""):
        print(f"[{self.groupname}] Bewege zu Pose: {label}")
        self.move_group.set_pose_target(pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def set_joint_target(self, joint_values):
        self.joint_goals.append(joint_values)


    def move(self):
        # Joint goals
        for joints in self.joint_goals:
            print(f"[{self.groupname}] Moving to joint target: {joints}")
            self.move_group.set_joint_value_target(joints)
            self.move_group.go(wait=True)
            self.move_group.stop()

        # Reset
        self.joint_goals = []

"""     def move(self):

        self.move_group.clear_pose_targets()
        try:
            for g in self.goals:
                self.move_group.set_pose_target(g)
                self.move_group.go(wait=True)
        except:
            print("Targets not reachable")
        finally:
            print("Stopping")
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            self.goals = []-




# Funktion anpassen! Und evtl. nicht verwenden -> konvertiert Koordinaten von einem reference frame in ein anderes frame (Welt in Roboter zB)
     def printPose(self,reference_frame=None):
        self.cp = self.move_group.get_current_pose()
        #print(self.move_group.get_pose_reference_frame())
        if reference_frame is None:
            print("Current pose for TCP <",self.groupname,"> in Reference frame <",self.cp.header.frame_id,"> :")
            print("x: ",self.cp.pose.position.x)
            print("y: ",self.cp.pose.position.y)
            print("z: ",self.cp.pose.position.z)
            print("qx: ",self.cp.pose.orientation.x)
            print("qy: ",self.cp.pose.orientation.y)
            print("qz: ",self.cp.pose.orientation.z)
            print("qw: ",self.cp.pose.orientation.w)
        else:
            #try:  
            self.tf_buffer.can_transform(reference_frame, self.cp.header.frame_id, rospy.Time(0), rospy.Duration(10.0))
            transformed_pose = self.tf_buffer.transform(self.cp, reference_frame)
            print("Current pose for TCP <",self.groupname,"> in Reference frame <",reference_frame,"> :")
            print("x: ",transformed_pose.pose.position.x)
            print("y: ",transformed_pose.pose.position.y)
            print("z: ",transformed_pose.pose.position.z)
            print("qx: ",transformed_pose.pose.orientation.x)
            print("qy: ",transformed_pose.pose.orientation.y)
            print("qz: ",transformed_pose.pose.orientation.z)
            print("qw: ",transformed_pose.pose.orientation.w)
            #except:
            #    print("Reference Frame does not exist!")

            # We get the joint values from the group and change some of the values:
        tau = 6.28
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 of a turn
        joint_goal[6] = 0

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

    
     def move(self):

        self.move_group.clear_pose_targets()
        try:
            for g in self.goals:
                self.move_group.set_pose_target(g)
                self.move_group.go(wait=True)
        except:
            print("Targets not reachable")
        finally:
            print("Stopping")
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            self.goals = []


     def set_pose_reference_frame(self, reference_frame):
            #Set the reference frame to assume for poses of end-effector
            self._g_g.set_pose_reference_frame(reference_frame) """
    
            

if __name__ == "__main__":
    
    # Roboter-Instanzen
    sixaxis = DemoRobot(groupname="sixaxis")
    sixaxis.initial_joint_positions = [0, 0, 0, 0, 0, 0, 0, 0]
    scara = DemoRobot(groupname="scara")
    scara.initial_joint_positions = [0, 0, 0, 0]


    # Zielposen für Six-Axis
    sa_poses = {
        # WERTE NICHT VERÄNDERN !!!!

        "sixaxis_home": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=-0.867, y=-1.121, z=1.591),
            orientation=geometry_msgs.msg.Quaternion(x=0.703, y=-0.001, z=0.712, w=0.000)
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


    # Zielposen für SCARA
    s_poses = {
        # WERTE NICHT VERÄNDERN!!!!
        "scara_home": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=2.959, y=-0.638, z=1.2),
            orientation=geometry_msgs.msg.Quaternion(x=0.998, y=-0.065, z=0.000, w=0.000)
        ),

        "s_above_pick_up": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=2.55, y=-0.2, z=1.2),
            orientation=geometry_msgs.msg.Quaternion(x=0.998, y=-0.065, z=0.000, w=0.000)
        ),
        "s_pick_up": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=2.55, y=-0.2, z=1.15),
            orientation=geometry_msgs.msg.Quaternion(x=0.998, y=-0.065, z=0.000, w=0.000)
        ),
        "s_above_place": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=2.469, y=-1.064, z=1.243),
            orientation=geometry_msgs.msg.Quaternion(x=0.998, y=-0.065, z=0.000, w=0.000)
        ),
        "s_place": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=2.469, y=-1.064, z=1.15),
            orientation=geometry_msgs.msg.Quaternion(x=0.998, y=-0.065, z=0.000, w=0.000)
        )
    }


    # SCARA

    print("\n. \n. \n")
    scara.move_to_pose(s_poses["scara_home"], "SCARA home")
    scara.get_current_pose()

    print("\n. \n. \n")
    scara.move_to_pose(s_poses["s_above_pick_up"], "SCARA above_pick_up")
    scara.get_current_pose()

    print("\n. \n. \n")
    scara.move_to_pose(s_poses["s_pick_up"], "SCARA pick_up")
    scara.get_current_pose()

    print("\n. \n. \n")
    scara.move_to_pose(s_poses["s_above_pick_up"], "SCARA above_pick_up")
    scara.get_current_pose()

    print("\n. \n. \n")
    scara.move_to_pose(s_poses["s_above_place"], "SCARA above_place")
    scara.get_current_pose()

    print("\n. \n. \n")
    scara.move_to_pose(s_poses["s_place"], "SCARA place")
    scara.get_current_pose()

    print("\n. \n. \n")
    scara.move_to_pose(s_poses["s_above_place"], "SCARA above_place")
    scara.get_current_pose()


    # SIXAXIS
    print("\n. \n. \n")
    sixaxis.get_current_pose()
    print("\n. \n. \n")
    sixaxis.set_joint_target([0, 0, 0, 0, 0, 0])                                    # home
    sixaxis.move()
    print("\n. \n. \n")
    sixaxis.set_joint_target([0.0873, 0.6632, 0.9948, 0.0, -1.850, 0.0698])         # above pick up
    sixaxis.move()
    print("\n. \n. \n")
    sixaxis.set_joint_target([0.0524, 0.6283, 0.4014, 0.0, -1.3439, 0.0524])        # pick up
    sixaxis.move()
    print("\n. \n. \n")
    sixaxis.set_joint_target([0.0873, 0.6632, 0.9948, 0.0, -1.850, 0.0698])         # above pick up
    sixaxis.move()
    print("\n. \n. \n")
    sixaxis.set_joint_target([3.0194, -0.0698, -0.4712, 0.0, -1.1519, -0.1222])     # above place
    sixaxis.move()
    print("\n. \n. \n")
    sixaxis.set_joint_target([3.129, 0.963, -0.204, 0.007, -0.400, -0.085])         # place
    sixaxis.move()
    print("\n. \n. \n")
    sixaxis.set_joint_target([3.0194, -0.0698, -0.4712, 0.0, -1.1519, -0.1222])     # above place
    sixaxis.move()
    print("\n. \n. \n")
    sixaxis.set_joint_target([0, 0, 0, 0, 0, 0])                                    # home
    sixaxis.move()
    print("\n. \n. \n")
    

"""
    print("\n. \n. \n")
    sixaxis.move_to_pose(sa_poses["sixaxis_home"], "SIXAXIS home")
    sixaxis.get_current_pose()

    print("\n. \n. \n")
    sixaxis.move_to_pose(sa_poses["sa_above_pick_up"], "SIXAXIS above_pick_up")
    sixaxis.get_current_pose()

    tau = 2*pi
    joint_goal = sixaxis.move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -tau / 8
    joint_goal[2] = 0
    joint_goal[3] = -tau / 4
    joint_goal[4] = 0
    joint_goal[5] = tau / 6  # 1/6 of a turn


    print("\n. \n. \n")
    sixaxis.move_to_pose(sa_poses["sa_above_pick_up"], "SIXAXIS above_pick_up")
    sixaxis.get_current_pose()
    
    print("\n. \n. \n")
    sixaxis.move_to_pose(sa_poses["sa_pick_up"], "SIXAXIS pick_up")
    sixaxis.get_current_pose()

    print("\n. \n. \n")
    sixaxis.move_to_pose(sa_poses["sa_above_pick_up"], "SIXAXIS above_pick_up")
    sixaxis.get_current_pose()

    print("\n. \n. \n")
    sixaxis.move_to_pose(sa_poses["sa_above_place"], "SIXAXIS above_place")
    sixaxis.get_current_pose()
    
    print("\n. \n. \n")
    sixaxis.move_to_pose(sa_poses["sa_place"], "SIXAXIS place")
    sixaxis.get_current_pose()

    print("\n. \n. \n")
    sixaxis.move_to_pose(sa_poses["sa_above_place"], "SIXAXIS above_place")
    sixaxis.get_current_pose()


    print("\n. \n. \n")
    sixaxis.move_to_pose(sa_poses["sa_above_pick_up"], "SIXAXIS above_pick_up")
    sixaxis.get_current_pose()

    print("\n. \n. \n")
    sixaxis.move_to_pose(sa_poses["sa_pick_up"], "SIXAXIS pick_up")
    sixaxis.get_current_pose()

    sixaxis.move_to_pose(sa_poses["pre_post_pick_up"], "SixAxis pre_post_pick_up")
    sixaxis.move_to_pose(sa_poses["pick_up"], "SixAxis pick_up")
    sixaxis.move_to_pose(sa_poses["pre_post_pick_up"], "SixAxis pre_post_pick_up")
    sixaxis.move_to_pose(sa_poses["pre_post_place"], "SixAxis pre_post_place")
    sixaxis.move_to_pose(sa_poses["place"], "SixAxis place")
    sixaxis.move_to_pose(sa_poses["pre_post_place"], "SixAxis pre_post_place")
    

    sixaxis.get_current_pose()
    sixaxis.move_to_pose(sa_poses["test"], "SIXAXIS home")
    sixaxis.get_current_pose()

    print("\n. \n. \n")
    sixaxis.move_to_pose(sa_poses["test2"], "SIXAXIS above_pick_up")
    sixaxis.get_current_pose() 


    print("\n. \n. \n")
    sixaxis.move_to_pose(sa_poses["sixaxis_home"], "SIXAXIS home")
    sixaxis.get_current_pose() """