#!/usr/bin/env python3
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np
import sys
import math

class demo_robot:
    def __init__(self, nodename="demo_robot", groupname="igus_robot_arm"):
        rospy.init_node(nodename, anonymous=False)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander(groupname)
        
        # Acceleration and Speed factors
        self.move_group.set_max_acceleration_scaling_factor(1)
        self.move_group.set_max_velocity_scaling_factor(1)
        self.move_group.set_goal_tolerance(0.001)
        self.goals = []

    def getPoseAndPrint(self):
        self.cp = self.move_group.get_current_pose().pose
        print("Info: current pose ")
        print("x: ", self.cp.position.x)
        print("y: ", self.cp.position.y)
        print("z: ", self.cp.position.z)
        print("qx: ", self.cp.orientation.x)
        print("qy: ", self.cp.orientation.y)
        print("qz: ", self.cp.orientation.z)
        print("qw: ", self.cp.orientation.w)

    def setTarget(self, x, y, z, roll, pitch, yaw):
        goal = geometry_msgs.msg.Pose()
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        goal.position.x = x
        goal.position.y = y
        goal.position.z = z
        goal.orientation.x = qx
        goal.orientation.y = qy
        goal.orientation.z = qz
        goal.orientation.w = qw
        try:
            self.goals.append(copy.deepcopy(goal))
        except:
            print("Target not set")

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
    def move_circle_xy(self, mx, my, mz, radius, n_punkte, roll, pitch, yaw):
        """
        Fährt eine Kreisbahn im XY (bei festem Z) mit kartesischem Pfad.
        """
        # Quaternion berechnen
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        waypoints = []
        for i in range(n_punkte + 1):
            winkel = 2 * math.pi * i / n_punkte
            x = mx + radius * math.cos(winkel)
            y = my + radius * math.sin(winkel)
            z = mz
            pose = geometry_msgs.msg.Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            waypoints.append(copy.deepcopy(pose))

        self.move_group.clear_path_constraints()
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,
            0.01,    # eef_step
            True     # avoid_collisions (bei deiner MoveIt-Version)
        )
        print(f"Kreisplanung: {fraction*100:.1f}% Pfad gefunden")
        self.move_group.execute(plan, wait=True)
        #if fraction > 0.99:
        #    self.move_group.execute(plan, wait=True)
        #    print("Kreisbewegung ausgeführt!")
        #else:
        #    print("Achtung: Pfad konnte nicht komplett geplant werden.")

        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def go_home(self):
        print("Fahre zur home_pose...")
        self.move_group.set_named_target("home_pose")
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def go_to_named_pose(self, pose_name):
        """
        Fahre eine im SRDF definierte 'named target' Pose an.
        :param pose_name: Name der Zielpose als String, z.B. 'home_pose' oder 'ready_pose'
        """
        print(f"Fahre zur benannten Pose: {pose_name}")
        self.move_group.set_named_target(pose_name)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        if success:
            print(f"Pose '{pose_name}' erfolgreich erreicht.")
        else:
            print(f"Fehler: Pose '{pose_name}' konnte nicht erreicht werden!")
        return success


if __name__ == "__main__":
    scara = demo_robot(nodename="demo_robot",groupname="scara_group")

    scara.go_to_named_pose("scara_home")

    # Start orientation of scara
    #scara_orientation = [3.142, -0.000, 0.848]
    # Pick up scara
    #scara_position = [6.770, -0.678, 1.224]
    
    #scara.setTarget(*scara_position, *scara_orientation)
    #scara.move()

    del scara

        # === Globale Parameter für IGUS-Arm ===
    # radius = 0.173
    # igus_orientation = [-math.pi, 0, -math.pi/2.0]
    # igus_object_x = -0.455
    # igus_object_y = 1.040
    # igus_object_z = 0.92
    # igus_circle_points = 40
# 
    # # === Globale Parameter für FANUC-Arm ===
    # fanuc_orientation = [-math.pi, 0, -math.pi]
    # fanuc_object_x = -0.455
    # fanuc_object_y = 2.239
    # fanuc_object_z = 1.0
    # fanuc_circle_points = 40
# 
    # # === IGUS Roboter ===
    # igus = demo_robot(nodename="demo_robot", groupname="igus_robot_arm")
    # igus.go_to_named_pose("ready_pose")
    # igus.move_circle_xy(
    #     mx=igus_object_x,
    #     my=igus_object_y,
    #     mz=igus_object_z,
    #     radius=radius,
    #     n_punkte=igus_circle_points,
    #     roll=igus_orientation[0],
    #     pitch=igus_orientation[1],
    #     yaw=igus_orientation[2]
    # )
    # igus.go_to_named_pose("home_pose")
# 
    # # === RIM 1 ===
    # rim_1 = demo_robot(nodename="demo_robot", groupname="rim_conveyor_1")
    # rim_1.go_to_named_pose("robot_2_pose")
# 
    # # === FANUC Roboter ===
    # fanuc = demo_robot(nodename="demo_robot", groupname="fanuc_robot_arm")
    # fanuc.move_circle_xy(
    #     mx=fanuc_object_x,
    #     my=fanuc_object_y,
    #     mz=fanuc_object_z,
    #     radius=radius,
    #     n_punkte=fanuc_circle_points,
    #     roll=fanuc_orientation[0],
    #     pitch=fanuc_orientation[1],
    #     yaw=fanuc_orientation[2]
    # )
    # fanuc.go_to_named_pose("home_pose")
    # rim_1.go_to_named_pose("finish_pose")
# 
    # # === RIM 2 ===
    # rim_2 = demo_robot(nodename="demo_robot", groupname="rim_conveyor_2")
    # rim_2.go_to_named_pose("robot_1_pose")
    # igus.move_circle_xy(
    #     mx=igus_object_x,
    #     my=igus_object_y,
    #     mz=igus_object_z,
    #     radius=radius,
    #     n_punkte=igus_circle_points,
    #     roll=igus_orientation[0],
    #     pitch=igus_orientation[1],
    #     yaw=igus_orientation[2]
    # )
    # igus.go_to_named_pose("home_pose")
    # rim_2.go_to_named_pose("robot_2_pose")
    # fanuc.move_circle_xy(
    #     mx=fanuc_object_x,
    #     my=fanuc_object_y,
    #     mz=fanuc_object_z,
    #     radius=radius,
    #     n_punkte=fanuc_circle_points,
    #     roll=fanuc_orientation[0],
    #     pitch=fanuc_orientation[1],
    #     yaw=fanuc_orientation[2]
    # )
    # fanuc.go_to_named_pose("home_pose")
    # rim_2.go_to_named_pose("finish_pose")
# 
    # # === RIM 3 ===
    # rim_3 = demo_robot(nodename="demo_robot", groupname="rim_conveyor_3")
    # rim_3.go_to_named_pose("robot_1_pose")
    # igus.move_circle_xy(
    #     mx=igus_object_x,
    #     my=igus_object_y,
    #     mz=igus_object_z,
    #     radius=radius,
    #     n_punkte=igus_circle_points,
    #     roll=igus_orientation[0],
    #     pitch=igus_orientation[1],
    #     yaw=igus_orientation[2]
    # )
    # igus.go_to_named_pose("home_pose")
    # rim_3.go_to_named_pose("finish_pose")
    # fanuc.move_circle_xy(
    #     mx=fanuc_object_x,
    #     my=fanuc_object_y,
    #     mz=fanuc_object_z,
    #     radius=radius,
    #     n_punkte=fanuc_circle_points,
    #     roll=fanuc_orientation[0],
    #     pitch=fanuc_orientation[1],
    #     yaw=fanuc_orientation[2]
    # )
    # fanuc.go_to_named_pose("home_pose")