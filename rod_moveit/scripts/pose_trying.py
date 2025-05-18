#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

class DemoRobot:
    def __init__(self, groupname):
        self.move_group = moveit_commander.MoveGroupCommander(groupname)
        self.groupname = groupname

        self.move_group.set_max_acceleration_scaling_factor(1.0)
        self.move_group.set_max_velocity_scaling_factor(1.0)
        self.move_group.set_goal_tolerance(0.001)

        self.pose_goals = []
        self.joint_goals = []

    def get_current_pose(self):
        pose = self.move_group.get_current_pose().pose
        print(f"[{self.groupname}] Current Pose:")
        print(f"  Position: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
        print(f"  Orientation: x={pose.orientation.x:.3f}, y={pose.orientation.y:.3f}, "
              f"z={pose.orientation.z:.3f}, w={pose.orientation.w:.3f}")
        return pose

    def set_pose_target(self, x, y, z, roll, pitch, yaw):
        pose_stamped = geometry_msgs.msg.PoseStamped()

        # RPY → Quaternion
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        # Setze Pose
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation.x = qx
        pose_stamped.pose.orientation.y = qy
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = qw

        # Setze das Referenzkoordinatensystem je nach Roboter
        if self.groupname == "scara":
            pose_stamped.header.frame_id = "scara_basejoint"
        elif self.groupname == "sixaxis":
            pose_stamped.header.frame_id = "sixaxis_basejoint"
        else:
            pose_stamped.header.frame_id = "base_link"

        # Zielpose speichern
        self.pose_goals.append(copy.deepcopy(pose_stamped))



    def set_joint_target(self, joint_values):
        self.joint_goals.append(joint_values)

    def move(self):
        # Joint goals
        for joints in self.joint_goals:
            print(f"[{self.groupname}] Moving to joint target: {joints}")
            self.move_group.go(joints, wait=True)
            self.move_group.stop()

        # Pose goals
        for pose in self.pose_goals:
            print(f"[{self.groupname}] Moving to pose target...")

            # Setze Planungskonfiguration
            self.move_group.set_planning_time(2.0)  # z. B. 10 Sekunden Planungszeit
            self.move_group.set_num_planning_attempts(10)  # z. B. 10 Versuche

            self.move_group.set_pose_target(pose)
            success = self.move_group.go(wait=True)

            if not success:
                rospy.logwarn(f"[{self.groupname}] Bewegung zum Pose-Ziel fehlgeschlagen!")

            self.move_group.stop()
            self.move_group.clear_pose_targets()

        # Reset
        self.joint_goals = []
        self.pose_goals = []


if __name__ == "__main__":
    rospy.init_node("multi_robot_demo", anonymous=False)
    moveit_commander.roscpp_initialize(sys.argv)

## Link Attacher (aus attach.py aus https://github.com/pal-robotics/gazebo_ros_link_attacher)
#    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
#    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
#                                    Attach)
#    attach_srv.wait_for_service()
#    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")
    
## Link Detacher (aus detach.py aus https://github.com/pal-robotics/gazebo_ros_link_attacher)
#    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
#    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
#                                    Attach)
#    detach_srv.wait_for_service()
#    rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")
    

    # Roboter-Instanzen um die verschiedenen movegroups anzusprechen
    scara = DemoRobot(groupname="scara")
    sixaxis = DemoRobot(groupname="sixaxis")
    
        #ATTACH Smartphone_Box an Drehtisch (code aus https://github.com/pal-robotics/gazebo_ros_link_attacher)
#    rospy.loginfo("Attaching Smartphone_Box and drehtisch")
#    req = AttachRequest()
#    req.model_name_1 = "Smartphone_Box"
#    req.link_name_1 = "link"
#    req.model_name_2 = "robot"
#    req.link_name_2 = "link1_d"
#    attach_srv.call(req)
        
    # Joint-Ziel für scara
    sixaxis.set_joint_target([0.7, 0.0, 0.0, 0.0])
    #Bewegung ausführen
    sixaxis.move()

    # Joint-Ziel für rothbot
    scara.set_joint_target([1.04, -1.76, 0.05, -0.05, 0.0, -0.35])
    scara.set_joint_target([1.04, -1.76, 0.05, -0.05, 0.145, -0.35])
    scara.move()
    
        # ATTACH Smartphone an rothbot
#    rospy.loginfo("Attaching Smartphone and rothbot")
#    req = AttachRequest()
#    req.model_name_1 = "Smartphone"
#    req.link_name_1 = "link"
#    req.model_name_2 = "robot"
#    req.link_name_2 = "link6_r"
#    attach_srv.call(req)
#    rospy.sleep(0.5)

    scara.set_joint_target([1.04, -1.76, 0.05, -0.05, 0.0, -0.35])
    scara.set_joint_target([-1.05, -0.80, -0.01, 0.01, 0.0, -0.282])
    # Hier einmal ein pose_target wollte anfangs pose_targets machen habe mich umentschienden joint targets zu verwenden weil es einfacher ist richitge positionen zu setzen (in diesem fall)
    # Implementierung ist trozdem vorhanden diese ezile macht genau das gleiche wie das joint target darunter
    #rothbot.set_pose_target(0.00, 0.26, 1.47, 3.14, 0.0, 1.57) #Habe pose_targets im code nur ienmal zum funktionieren gebracht, Im hmi konnte ich die position erreichen
    scara.set_joint_target([-1.05, -0.80, -0.01, 0.01, 0.02, -0.282])
    scara.move()
    
        # DETACH Smartphone von rothbot
#    rospy.loginfo("Detach Smartphone and rothbot")
#    req = AttachRequest()
#    req.model_name_1 = "Smartphone"
#    req.link_name_1 = "link"
#    req.model_name_2 = "robot"
#    req.link_name_2 = "link6_r"
#    detach_srv.call(req)
#    rospy.sleep(0.5)
    
        # ATTACH Smartphone an Smartphone_box
#    rospy.loginfo("Attaching Smartphone and Smartphone_box")
#    req = AttachRequest()
#    req.model_name_1 = "Smartphone"
#    req.link_name_1 = "link"
#    req.model_name_2 = "Smartphone_Box"
#    req.link_name_2 = "link"
#    attach_srv.call(req)
#    rospy.sleep(0.5)

    scara.set_joint_target([-1.05, -0.80, -0.01, 0.01, 0.0, -0.282])
    scara.set_joint_target([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # Bewegung ausführen
    scara.move()

    # Joint-Ziel für drehtisch
#    drehtisch.set_joint_target([3.14])  # 180°
#    drehtisch.move()

    # Joint-Ziel für sixaxis
    sixaxis.set_joint_target([0.0, 0.07, 0.0, -0.1])
    sixaxis.move()
    
        # DETACH Smartphone_Box von drehtisch
#    rospy.loginfo("Detach Smartphone_Box and rothbot")
#    req = AttachRequest()
#    req.model_name_1 = "Smartphone_Box"
#    req.link_name_1 = "link"
#    req.model_name_2 = "robot"
#    req.link_name_2 = "link1_d"
#    detach_srv.call(req)
#    rospy.sleep(0.5)
    
        # ATTACH Smartphone_Box an sixaxis
#    rospy.loginfo("Attaching Smartphone_Box and hunfagltron")
#    req = AttachRequest()
#    req.model_name_1 = "Smartphone_Box"
#    req.link_name_1 = "link"
#    req.model_name_2 = "robot"
#    req.link_name_2 = "link4_h"
#    attach_srv.call(req)
#    rospy.sleep(0.5)

    sixaxis.set_joint_target([1.57, 0.0, 0.0, 0.0])
    sixaxis.set_joint_target([1.57, -0.47, -0.53, 0.47])
    sixaxis.move()
    
        # DETACH von sixaxis
#    rospy.loginfo("Attaching Smartphone_Box and hunfagltron")
#    req = AttachRequest()
#    req.model_name_1 = "Smartphone_Box"
#    req.link_name_1 = "link"
#    req.model_name_2 = "robot"
#    req.link_name_2 = "link4_h"
#    detach_srv.call(req)
#    rospy.sleep(0.5)

    sixaxis.set_joint_target([1.57, 0.0, 0.0, 0.0])
    sixaxis.set_joint_target([0.0, 0.07, 0.0, -0.1])
    sixaxis.set_joint_target([-1.57, 0.0, 0.0, 0.0])
    sixaxis.set_joint_target([-1.57, -0.47, -0.53, 0.47])
    sixaxis.move()
    
        # ATTACH an sixaxis
#    rospy.loginfo("Attaching Smartphone_Box and hunfagltron")
#    req = AttachRequest()
#    req.model_name_1 = "Smartphone_Box_neu"
#    req.link_name_1 = "link"
#    req.model_name_2 = "robot"
#    req.link_name_2 = "link4_h"
#    attach_srv.call(req)
#    rospy.sleep(0.5)

    sixaxis.set_joint_target([-1.57, 0.0, 0.0, 0.0])
    sixaxis.set_joint_target([0.0, 0.07, 0.0, -0.1])
    sixaxis.move()
    
        # DETACH von sixaxis
#    rospy.loginfo("Attaching Smartphone_Box and hunfagltron")
#    req = AttachRequest()
#    req.model_name_1 = "Smartphone_Box_neu"
#    req.link_name_1 = "link"
#    req.model_name_2 = "robot"
#    req.link_name_2 = "link4_h"
#    detach_srv.call(req)
#    rospy.sleep(0.5)

    sixaxis.set_joint_target([0.7, 0.0, 0.0, 0.0])
    sixaxis.move()

        # ATTACH an drehtisch
#    rospy.loginfo("Attaching Smartphone_Box_neu and drehtisch")
#    req = AttachRequest()
#    req.model_name_1 = "Smartphone_Box_neu"
#    req.link_name_1 = "link"
#    req.model_name_2 = "robot"
#    req.link_name_2 = "link1_d"
#    attach_srv.call(req)
#    rospy.sleep(0.5)



    # Joint-Ziel für drehtisch
#    drehtisch.set_joint_target([0.0])  # 0°
#    drehtisch.move()
    
