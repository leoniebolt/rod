#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import moveit_commander
from tf.transformations import quaternion_from_euler

def move_tcp(direction):
    pose_target = group.get_current_pose().pose
    waypoints = []

    step_size = 0.05  # 5 cm

    if direction == "up":
        pose_target.position.z += step_size
    elif direction == "down":
        pose_target.position.z -= step_size
    elif direction == "left":
        pose_target.position.y += step_size
    elif direction == "right":
        pose_target.position.y -= step_size
    elif direction == "forward":
        pose_target.position.x += step_size
    elif direction == "backward":
        pose_target.position.x -= step_size
    else:
        rospy.logwarn("[UR] Ungültige Richtung: %s", direction)
        return

    # Orientierung bleibt gleich
    q = quaternion_from_euler(0, 0, 0)
    pose_target.orientation.x = q[0]
    pose_target.orientation.y = q[1]
    pose_target.orientation.z = q[2]
    pose_target.orientation.w = q[3]

    waypoints.append(pose_target)

    plan, fraction = group.compute_cartesian_path(
        waypoints,
        eef_step=0.01,
        jump_threshold=0.0,
        avoid_collisions=False
    )

    if fraction < 0.9:
        rospy.logwarn("[UR] IK fehlgeschlagen für '%s'. Nur %f erreicht.", direction, fraction)
    else:
        group.execute(plan, wait=True)
        rospy.loginfo("[UR] Bewegung in Richtung '%s' ausgeführt.", direction)

def callback(msg):
    if msg.data == "stop":
        group.stop()
        rospy.loginfo("Bewegung gestoppt.")
    else:
        move_tcp(msg.data)

if __name__ == '__main__':
    moveit_commander.roscpp_initialize([])
    rospy.init_node('control_listener_ur', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "sixaxis"  # Achtung: Muss zur move_group passen
    group = moveit_commander.MoveGroupCommander(group_name)

    rospy.loginfo("Ready to take commands for planning group %s.", group_name)
    rospy.Subscriber("/robot_control_topic", String, callback)
    rospy.loginfo("UR Listener bereit.")
    rospy.spin()
