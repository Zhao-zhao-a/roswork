#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

def odom_callback(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    # Time and frame assignments
    t.header.stamp = msg.header.stamp
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"

    # Position from ground truth
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z

    # Orientation from ground truth
    t.transform.rotation = msg.pose.pose.orientation

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('gazebo_odom_tf_broadcaster')
    # Subscribe to Gazebo P3D plugin ground truth state
    rospy.Subscriber('/ground_truth/state', Odometry, odom_callback)
    rospy.spin()
