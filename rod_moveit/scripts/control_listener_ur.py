#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import moveit_commander
import geometry_msgs.msg
import copy

step_size = 0.001  # 5cm pro Befehl

def move_tcp(direction):
    current_pose = group.get_current_pose().pose
    target_pose = geometry_msgs.msg.Pose()
    target_pose.orientation = current_pose.orientation  # Orientierung beibehalten

    # Ziel-Position setzen
    target_pose.position = current_pose.position

    if direction == "up":
        target_pose.position.z += step_size
    elif direction == "down":
        target_pose.position.z -= step_size
    elif direction == "left":
        target_pose.position.y += step_size
    elif direction == "right":
        target_pose.position.y -= step_size
    elif direction == "forward":
        target_pose.position.x += step_size
    elif direction == "backward":
        target_pose.position.x -= step_size
    elif direction == "stop":
        rospy.loginfo("Bewegung gestoppt.")
        return


    # IK: Lineare Bewegung mit compute_cartesian_path
    waypoints = [target_pose]
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,
        eef_step=0.01,       # Auflösung: 1 cm Schritte
    )
    
    group.execute(plan, wait=True)
    rospy.loginfo(f"[UR] Bewegung '{direction}' ausgeführt.")

def callback(msg):
    print("test")
    move_tcp(msg.data)

if __name__ == '__main__':
    rospy.init_node('control_listener_ur')
    moveit_commander.roscpp_initialize([])
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("sixaxis")  # Passe das ggf. an deinen MoveGroup-Namen an
    group.set_pose_reference_frame("sixaxis_j6")

    rospy.Subscriber('/ur_control_topic', String, callback)
    rospy.loginfo("UR Listener bereit.")
    rospy.spin()
