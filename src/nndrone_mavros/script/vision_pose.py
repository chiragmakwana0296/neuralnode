#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped


rospy.init_node("vision_pose_pub")

def odom_cb(msg):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose = msg.pose.pose
    pose_pub.publish(pose)

    twist = TwistStamped()
    twist.header.stamp = rospy.Time.now()
    twist.twist = msg.twist.twist
    vel_pub.publish(twist)


if __name__ == '__main__':
    rospy.init_node("vision_pose_pub")
    pose_pub = rospy.Publisher("/robot_pose", PoseStamped, queue_size=10)
    vel_pub = rospy.Publisher("/robot_vel", TwistStamped, queue_size=10)
    sub = rospy.Subscriber("/odom_topic", Odometry, odom_cb)
    rospy.spin()
    
