#!/usr/bin/env python3
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize
import rospy
import sys
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

roscpp_initialize(sys.argv)
rospy.init_node('set_current_pose_as_target', anonymous=True)

robot = RobotCommander()
group = MoveGroupCommander("sixaxis")
print("Active joints:", group.get_active_joints())

# Set tighter goal tolerances to avoid GOAL_TOLERANCE_VIOLATED
group.set_goal_tolerance(1e-1)
group.set_goal_joint_tolerance(1e-1)
group.set_goal_position_tolerance(1e-1)
group.set_goal_orientation_tolerance(1e-1)

# Get current pose
current_pose = group.get_current_pose().pose
print("Current pose:", current_pose)

# Set start state explicitly
group.set_start_state_to_current_state()

# Try to set it as target
group.set_pose_target(current_pose)
success = group.go(wait=True)

if success:
    print("[INFO] Target pose reached (even though it's the same).")
else:
    print("[WARN] MoveIt rejected the target pose or planning failed.")

group.stop()
group.clear_pose_targets()