#!/usr/bin/env python3


## @file control_listener_sixaxis.py
#  @brief ROS node that listens to commands and moves a robot (e.g. UR5) linearly in Cartesian space using MoveIt.
#  @details Supports movement in 6 directions and stops via simple string-based commands.  
#  Uses compute_cartesian_path for planning with a fixed step size.

import rospy
from std_msgs.msg import String
import moveit_commander
import geometry_msgs.msg
import copy

## Step size in meters (5 cm per command step)
step_size = 0.05  # 5cm pro Befehl


## @brief Moves the robot's TCP in a specific direction using Cartesian planning.
#  @param direction String command indicating movement direction: 'up', 'down', 'left', 'right', 'forward', 'backward', or 'stop'.

def move_tcp(direction):
    # Create a target pose (relative to current pose, if needed)
    # current_pose = group.get_current_pose().pose

    # Create a target pose (relative to current pose, if needed)
    target_pose = geometry_msgs.msg.Pose()
    # target_pose.orientation = current_pose.orientation  # Orientierung beibehalten

    # Ziel-Position setzen
    # target_pose.position = current_pose.position

    ## Set linear movement along one axis
    if direction == "pos_x":
        target_pose.position.x += step_size
    elif direction == "neg_x":
        target_pose.position.x -= step_size
    elif direction == "pos_y":
        target_pose.position.y += step_size
    elif direction == "neg_y":
        target_pose.position.y -= step_size
    elif direction == "pos_z":
        target_pose.position.z += step_size
    elif direction == "neg_z":
        target_pose.position.z -= step_size
        return

    # Cartesian path planning
    # IK: Lineare Bewegung mit compute_cartesian_path
    print(target_pose)
    waypoints = [target_pose]
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,
        eef_step=0.01,       # Auflösung: 1 cm Schritte
    )

    # Execute trajectory
    group.set_pose_reference_frame("sixaxis_j6")
    group.execute(plan, wait=True)
    waypoints = []
    group.stop()
    group.clear_pose_targets()
    rospy.loginfo(f"[SIXAXIS] Bewegung '{direction}' ausgeführt.")

## @brief Callback function triggered by incoming String messages.
#  @param msg ROS message containing a movement command.    

def callback(msg):
    print("test")
    move_tcp(msg.data)


## @brief Main function initializing ROS node and MoveIt interfaces.
## Robot commander interface
if __name__ == '__main__':
    rospy.init_node('control_listener_sixaxis')
    moveit_commander.roscpp_initialize([])

    ## Robot commander interface
    robot = moveit_commander.RobotCommander()

    ## Planning scene interface
    scene = moveit_commander.PlanningSceneInterface()

    ## Move group commander for the sixaxis group
    group = moveit_commander.MoveGroupCommander("sixaxis_group")  
    
    ## Subscriber to control topic
    rospy.Subscriber('/sixaxis_control_topic', String, callback)
    
    rospy.loginfo("SIXAXIS Listener bereit.")
    rospy.spin()
