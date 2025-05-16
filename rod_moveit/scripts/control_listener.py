#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def callback(msg):
    command = msg.data
    twist = Twist()

    if command == 'up':
        twist.linear.x = 0.5
    elif command == 'down':
        twist.linear.x = -0.5
    elif command == 'left':
        twist.angular.z = 0.5
    elif command == 'right':
        twist.angular.z = -0.5
    elif command == 'stop':
        twist = Twist()  # alles auf 0

    pub.publish(twist)
    rospy.loginfo(f"Published Twist: {twist}")

rospy.init_node('control_listener')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.Subscriber('/robot_control_topic', String, callback)
rospy.spin()
