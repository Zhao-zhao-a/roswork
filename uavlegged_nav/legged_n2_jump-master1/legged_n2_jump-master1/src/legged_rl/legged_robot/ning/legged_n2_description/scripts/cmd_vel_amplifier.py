#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class CmdVelAmplifier:
    def __init__(self):
        self.scale_linear_x = rospy.get_param('~scale_linear_x', 2.0)
        self.scale_angular_z = rospy.get_param('~scale_angular_z', 2.0)
        self.min_linear_x = rospy.get_param('~min_linear_x', 0.4)
        self.min_angular_z = rospy.get_param('~min_angular_z', 0.4)
        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/nav_cmd_vel', Twist, self.callback)

    def callback(self, msg):
        out_msg = Twist()
        out_msg.linear.y = 0.0 # Force no lateral movement if not needed
        
        # Process linear x
        if abs(msg.linear.x) > 0.01:
            out_msg.linear.x = msg.linear.x * self.scale_linear_x
            if out_msg.linear.x > 0 and out_msg.linear.x < self.min_linear_x:
                out_msg.linear.x = self.min_linear_x
            elif out_msg.linear.x < 0 and out_msg.linear.x > -self.min_linear_x:
                out_msg.linear.x = -self.min_linear_x
        
        # Process angular z
        if abs(msg.angular.z) > 0.01:
            out_msg.angular.z = msg.angular.z * self.scale_angular_z
            if out_msg.angular.z > 0 and out_msg.angular.z < self.min_angular_z:
                out_msg.angular.z = self.min_angular_z
            elif out_msg.angular.z < 0 and out_msg.angular.z > -self.min_angular_z:
                out_msg.angular.z = -self.min_angular_z

        self.pub.publish(out_msg)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_amplifier')
    node = CmdVelAmplifier()
    rospy.spin()
