#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import moveit_commander

def move_tcp(direction):
    pose = group.get_current_pose().pose
    step = 0.05  # 5 cm Schrittgröße
    waypoints = []

    if direction == "up":
        pose.position.z += step
    elif direction == "down":
        pose.position.z -= step
    elif direction == "left":
        pose.position.y += step
    elif direction == "right":
        pose.position.y -= step
    elif direction == "forward":
        pose.position.x += step
    elif direction == "backward":
        pose.position.x -= step
    else:
        rospy.logwarn(f"[UR] Ungültiger Befehl: {direction}")
        return

    waypoints.append(pose)

    plan, fraction = group.compute_cartesian_path(
        waypoints,
        0.01  # eef_step
    )

    if fraction > 0.9:
        group.execute(plan, wait=True)
        rospy.loginfo(f"[UR] Bewegung '{direction}' erfolgreich ausgeführt (Fraktion: {fraction:.2f}).")
    else:
        rospy.logwarn(f"[UR] Cartesian Path unvollständig (Fraktion: {fraction:.2f}).")

def callback(msg):
    if msg.data == "stop":
        group.stop()
        rospy.loginfo("[UR] Bewegung gestoppt.")
    else:
        move_tcp(msg.data)

if __name__ == '__main__':
    moveit_commander.roscpp_initialize([])
    rospy.init_node('control_listener_ur', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("sixaxis")

    group.set_max_velocity_scaling_factor(0.1)
    group.set_max_acceleration_scaling_factor(0.1)

    rospy.Subscriber("/robot_control_topic", String, callback)
    rospy.loginfo("[UR] Steuerung bereit für Planungsgruppe 'sixaxis'")
    rospy.spin()
