#!/usr/bin/env python

import sys
import moveit_commander
import geometry_msgs.msg
import time
import rospy

# ROS Node initialisieren
rospy.init_node("moveit_demo", anonymous=True)

# MoveIt initialisieren
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
sa_group = moveit_commander.MoveGroupCommander("sixaxis")
s_group = moveit_commander.MoveGroupCommander("scara")

# SCARA Zielposen
s_poses = {
    "s_pre_post_pick_up_pose": geometry_msgs.msg.Pose(
        position=geometry_msgs.msg.Point(x=-1.2, y=-0.2, z=1.5),
        orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    ),
    "s_pick_up_pose": geometry_msgs.msg.Pose(
        position=geometry_msgs.msg.Point(x=-1.2, y=-0.2, z=1.0),
        orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    ),
    "s_pre_post_place_pose": geometry_msgs.msg.Pose(
        position=geometry_msgs.msg.Point(x=-1.2, y=-1.3, z=1.5),
        orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    ),
    "s_place_pose": geometry_msgs.msg.Pose(
        position=geometry_msgs.msg.Point(x=-1.2, y=-1.3, z=1.0),
        orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )
}

# SixAxis Zielposen
sa_poses = {
    "sa_pre_post_pick_up_pose": geometry_msgs.msg.Pose(
        position=geometry_msgs.msg.Point(x=-1.0, y=-1.0, z=1.5),
        orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=1.0, w=0.0)
    ),
    "sa_pick_up_pose": geometry_msgs.msg.Pose(
        position=geometry_msgs.msg.Point(x=-1.0, y=-1.0, z=1.0),
        orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=1.0, w=0.0)
    ),
    "sa_pre_post_place_pose": geometry_msgs.msg.Pose(
        position=geometry_msgs.msg.Point(x=-2.5, y=-1.0, z=1.0),
        orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=1.0, w=0.0)
    ),
    "sa_place_pose": geometry_msgs.msg.Pose(
        position=geometry_msgs.msg.Point(x=-2.5, y=-1.0, z=0.2),
        orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=1.0, w=0.0)
    )
}

# Funktion zum Bewegen eines MoveGroups zu einer Pose
def move_to_pose(group, pose, label):
    group.set_pose_target(pose)
    plan = group.plan()

    # Wenn Planung erfolgreich war (prüfe nur auf non-empty trajectory)
    if plan and hasattr(plan, 'joint_trajectory') and plan.joint_trajectory.points:
        success = group.execute(plan, wait=True)
        group.stop()
        group.clear_pose_targets()

        if success:
            print(f"✅ Erfolgreich bewegt zu {label}")
            print("→ Aktuelle Pose:", group.get_current_pose().pose)
        else:
            print(f"⚠️ Ausführung zu {label} fehlgeschlagen.")
    else:
        print(f"❌ Planung zu {label} fehlgeschlagen.")

    time.sleep(1)

# 1. SCARA fährt alle 4 Positionen einmal an
print("\n🚦 Starte SCARA Sequenz...")
for name, pose in s_poses.items():
    print(f"\n🟡 Scara → {name}")
    move_to_pose(s_group, pose, name)

# 2. Danach fährt SixAxis in einer Endlosschleife alle Positionen ab
print("\n🔁 Starte SixAxis Loop...")
while not rospy.is_shutdown():
    for name, pose in sa_poses.items():
        print(f"\n🟡 SixAxis → {name}")
        move_to_pose(sa_group, pose, name)
