#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import moveit_commander
import geometry_msgs.msg
import copy

step_size = 0.05  # 5 cm pro Befehl

def move_tcp(direction):
    current_pose = group.get_current_pose().pose
    target_pose = copy.deepcopy(current_pose)  # Sichere Kopie

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
    else:
        rospy.logwarn(f"[UR] Ungültiger Befehl: '{direction}'")
        return

    waypoints = [target_pose]
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)

    if fraction > 0.95:
        rospy.loginfo(f"[UR] Bewegung '{direction}' geplant (Fraction: {fraction:.2f}). Starte Ausführung...")
        group.execute(plan, wait=True)
    else:
        rospy.logwarn(f"[UR] IK fehlgeschlagen oder unvollständig für '{direction}' (Pfad-Fraction: {fraction:.2f})")

def callback(msg):
    move_tcp(msg.data)

if __name__ == '__main__':
    rospy.init_node('control_listener_ur')
    moveit_commander.roscpp_initialize([])
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("sixaxis")  
    group.set_pose_reference_frame("sixaxis_j6")             

    rospy.Subscriber('/ur_control_topic', String, callback)
    rospy.loginfo("UR Listener bereit.")
    rospy.spin()
