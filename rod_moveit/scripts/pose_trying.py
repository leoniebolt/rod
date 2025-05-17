#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('reachable_workspace_sampling', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "scara"  # oder "sixaxis"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Fixe Orientierung (nach oben)
orientation = geometry_msgs.msg.Quaternion()
orientation.w = 1.0  # Keine Drehung

print("🔍 Starte Sampling des Arbeitsbereichs...")

# Sampling-Parameter
x_range = [round(x * 0.01, 2) for x in range(10, 51, 5)]   # 0.10 bis 0.50 m
y_range = [round(y * 0.01, 2) for y in range(-30, 31, 10)]  # -0.30 bis +0.30 m
z_range = [0.1, 0.15, 0.2]  # fix oder wenige Werte

reachable_poses = []

for x in x_range:
    for y in y_range:
        for z in z_range:
            pose = geometry_msgs.msg.Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation = orientation

            move_group.set_pose_target(pose)
            plan_result = move_group.plan()

            # Bei neueren MoveIt-Versionen ist plan() ein Tupel
            if isinstance(plan_result, tuple):
                plan = plan_result[1]
            else:
                plan = plan_result

            if plan and hasattr(plan, "joint_trajectory") and plan.joint_trajectory.points:
                reachable_poses.append((x, y, z))
                print(f"✅ Erreichbar: x={x}, y={y}, z={z}")
            else:
                print(f"❌ Nicht erreichbar: x={x}, y={y}, z={z}")

move_group.clear_pose_targets()

print(f"\n🧮 Insgesamt erreichbare Posen: {len(reachable_poses)}")
