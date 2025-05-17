#!/usr/bin/env python

import sys
import moveit_commander
import geometry_msgs.msg
import time
import rospy

# Initialize ros node
rospy.init_node("moveit_demo", anonymous=True)

# Initialisierung
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("sixaxis")


# Zielposen definieren
poses = {
    "pre_post_pick_up_pose": geometry_msgs.msg.Pose(
        position=geometry_msgs.msg.Point(x=-1.0, y=-1.0, z=1.5),
        orientation=geometry_msgs.msg.Quaternion(w=-1.0)
        ),
    "pick_up_pose": geometry_msgs.msg.Pose(
        position=geometry_msgs.msg.Point(x=-1.0, y=-1.0, z=1.0),
        orientation=geometry_msgs.msg.Quaternion(w=-1.0)
    ),
    "pre_post_place_pose": geometry_msgs.msg.Pose(
        position=geometry_msgs.msg.Point(x=-2.5, y=-1.0, z=1.0),
        orientation=geometry_msgs.msg.Quaternion(w=-1.0)
    ),
    "place_pose": geometry_msgs.msg.Pose(
        position=geometry_msgs.msg.Point(x=-2.5, y=-1.0, z=0.2),
        orientation=geometry_msgs.msg.Quaternion(w=-1.0)
    )
}



""" 
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = -1.0
pose_target.position.y = -1.0
pose_target.position.z = 1.5
group.set_pose_target(pose_target)

# Planung und Ausführung
plan = group.go(wait=True)
group.stop()
group.clear_pose_targets() 
"""



print("Planning frame:", group.get_planning_frame())
print("Aktuelle Pose:", group.get_current_pose().pose)



for name in ["pre_post_pick_up_pose", "pick_up_pose", "pre_post_place_pose", "place_pose"]:
    print(f"\n🟡 Bewege zu: {name}")
    group.set_pose_target(poses[name])
    plan = group.plan()
    print(f"Plan success: {plan[0]}")
    success = group.execute(plan[1], wait=True)

    #success = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    if not success:
        print(f"⚠️ Bewegung zu {name} fehlgeschlagen.")
    else:
        print(f"✅ Erfolgreich bewegt zu {name}")
        print("→ Aktuelle Pose:", group.get_current_pose().pose)

    time.sleep(1)