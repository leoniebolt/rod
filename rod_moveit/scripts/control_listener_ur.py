#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

step_size = 0.01  # 1 cm pro Befehl

def move_tcp(direction):
    current_pose = group.get_current_pose().pose
    waypoints = []

    # Zielpose auf Basis aktueller Pose definieren
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = current_pose.position.x
    target_pose.position.y = current_pose.position.y
    target_pose.position.z = current_pose.position.z

    # Feste Orientierung setzen
    q = quaternion_from_euler(0, 0, 0)  # Roll, Pitch, Yaw
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]

    # Bewegungsrichtung setzen
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

    waypoints.append(target_pose)

    # Cartesian Path berechnen (ROS Noetic-kompatibel!)
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,
        0.01,   # eef_step = 1 cm
        0.0     # jump_threshold
    )

    if fraction > 0.9:
        group.execute(plan, wait=True)
        rospy.loginfo(f"[UR] Lineare Bewegung '{direction}' ausgeführt ({fraction*100:.1f}%).")
    else:
        rospy.logwarn(f"[UR] Lineare IK fehlgeschlagen für '{direction}', nur {fraction*100:.1f}% erreicht.")

def callback(msg):
    move_tcp(msg.data)

if __name__ == '__main__':
    rospy.init_node('control_listener_ur')
    moveit_commander.roscpp_initialize([])
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("sixaxis")  # ← Passe ggf. den Namen an

    rospy.Subscriber('/ur_control_topic', String, callback)
    rospy.loginfo("UR Listener bereit.")
    rospy.spin()
