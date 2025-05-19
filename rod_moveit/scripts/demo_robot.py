#!/usr/bin/env python3
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np
import sys


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
        pose_stamped.header.frame_id = "scara_basejoint"

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
    rospy.init_node("scara_node", anonymous=False)
    moveit_commander.roscpp_initialize(sys.argv)


    scara = DemoRobot(groupname="scara")

    scara.set_joint_target([1.04, -1.76, 0.05, -0.05])
    scara.move()
    
    scara.set_joint_target([-1.05, -0.80, 0.31, 0.0])
    scara.move()
    
    scara.set_joint_target([0.0, 0.0, 0.0, 0.0])
    scara.move()


"""""
class SimpleRobotMover:
    def __init__(self, groupname="scara"):
        rospy.init_node("scara_node", anonymous=False)
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander(groupname)

        # Bewegungsparameter
        self.move_group.set_max_acceleration_scaling_factor(1.0)
        self.move_group.set_max_velocity_scaling_factor(1.0)
        self.move_group.set_goal_tolerance(0.001)
        self.move_group.set_planning_time(10)

    def move_to_pose(self, x, y, z, roll, pitch, yaw):
        pose_target = geometry_msgs.msg.Pose()

        # Roll-Pitch-Yaw → Quaternion
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z
        pose_target.orientation.x = qx
        pose_target.orientation.y = qy
        pose_target.orientation.z = qz
        pose_target.orientation.w = qw

        # Punkt-zu-Punkt-Bewegung
        self.move_group.set_pose_target(pose_target)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Lineare Bewegung (nach oben auf Y-Achse als Beispiel)
        waypoints = []
        current_pose = self.move_group.get_current_pose().pose
        current_pose.position.y += 0.2
        waypoints.append(copy.deepcopy(current_pose))

        (path, _) = self.move_group.compute_cartesian_path(waypoints, 0.01, True)
        self.move_group.execute(path, wait=True)

    def shutdown(self):
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    mover = SimpleRobotMover(groupname="scara")

    # Beispielpositionen anfahren
    # mover.move_to_pose(-1.2, -0.2, 1.3, 0, 0, 0)   # pre/post pickup
    # mover.move_to_pose(-1.2, -0.2, 1.0, 0, 0, 0)   # pickup
    # mover.move_to_pose(-1.2, -1.3, 1.3, 0, 0, 0)   # pre/post place
    # mover.move_to_pose(-1.2, -1.3, 1.0, 0, 0, 0)   # place

    # Aktuelle Werte einmal lesen
    joint_goal = mover.move_group.get_current_joint_values()

    # Erstes Gelenk auf 1.0
    joint_goal[0] = 1.0
    print("Fahre Gelenk 0 auf:", joint_goal)
    mover.move_group.set_joint_value_target(joint_goal)
    mover.move_group.go(wait=True)
    mover.move_group.stop()

    # Jetzt NICHT erneut get_current_joint_values() holen,
    # sondern das vorhandene joint_goal anpassen:
    joint_goal[1] = 1.0
    print("Fahre Gelenk 1 auf:", joint_goal)
    mover.move_group.set_joint_value_target(joint_goal)
    mover.move_group.go(wait=True)
    mover.move_group.stop()

    # mover.shutdown()




"""""