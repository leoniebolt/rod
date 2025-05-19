#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np

class DemoRobot:
    
    
    def __init__(self, nodename="DemoRobot", groupname="robotGroup"):
        rospy.init_node(nodename, anonymous=False)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander(groupname)
        
        #Acceleration and Speed factors
        self.move_group.set_max_acceleration_scaling_factor(1)
        self.move_group.set_max_velocity_scaling_factor(1)
        
        self.move_group.set_goal_tolerance(0.5)
        self.goals = []
        
        self.move_group = moveit_commander.MoveGroupCommander(groupname)
        self.groupname = groupname

        self.move_group.set_max_acceleration_scaling_factor(1.0)
        self.move_group.set_max_velocity_scaling_factor(1.0)
        self.move_group.set_goal_tolerance(0.001)


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
        """Set the reference frame to assume for poses of end-effectors"""
        self._g_g.set_pose_reference_frame(reference_frame)

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


if __name__ == "__main__":
    
    # Roboter-Instanzen
    sixaxis = DemoRobot(groupname="sixaxis")
    scara = DemoRobot(groupname="scara")


    # Zielposen für Six-Axis
    sa_poses = {
        # WERTE NICHT VERÄNDERN !!!!
        "sixaxis_home": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=-0.864, y=-1.123, z=1.584),
            orientation=geometry_msgs.msg.Quaternion(x=0.701, y=-0.002, z=0.713, w=0.000)
        ),

        "sa_above_pick_up": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=-0.868, y=-1.087, z=1.369),
            orientation=geometry_msgs.msg.Quaternion(x=0.999, y=0.016, z=0.030, w=0.001)
        ),
        "sa_pick_up": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=-0.858, y=-1.087, z=1.198),
            orientation=geometry_msgs.msg.Quaternion(x=-1.000, y=-0.012, z=-0.022, w=0.005)
        ),
        "sa_above_place": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=-2.425, y=-1.040, z=1.067),
            orientation=geometry_msgs.msg.Quaternion(x=-0.715, y=-0.699, z=0.004, w=0.012)
        ),
        "sa_place": geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=-2.771, y=-1.029, z=0.421),
            orientation=geometry_msgs.msg.Quaternion(x=-0.717, y=-0.697, z=0.007, w=0.018)
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

    """
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

    
"""


    # SIXAXIS
    sixaxis.get_current_pose()


    print("\n. \n. \n")
    sixaxis.move_to_pose(sa_poses["sixaxis_home"], "SIXAXIS home")
    sixaxis.get_current_pose()

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


    #print("\n. \n. \n")
    #sixaxis.move_to_pose(sa_poses["sa_above_pick_up"], "SIXAXIS above_pick_up")
    #sixaxis.get_current_pose()

    #print("\n. \n. \n")
    #sixaxis.move_to_pose(sa_poses["sa_pick_up"], "SIXAXIS pick_up")
    #sixaxis.get_current_pose()

    #sixaxis.move_to_pose(sa_poses["pre_post_pick_up"], "SixAxis pre_post_pick_up")
    #sixaxis.move_to_pose(sa_poses["pick_up"], "SixAxis pick_up")
    #sixaxis.move_to_pose(sa_poses["pre_post_pick_up"], "SixAxis pre_post_pick_up")
    #sixaxis.move_to_pose(sa_poses["pre_post_place"], "SixAxis pre_post_place")
    #sixaxis.move_to_pose(sa_poses["place"], "SixAxis place")
    #sixaxis.move_to_pose(sa_poses["pre_post_place"], "SixAxis pre_post_place")
    """

"""