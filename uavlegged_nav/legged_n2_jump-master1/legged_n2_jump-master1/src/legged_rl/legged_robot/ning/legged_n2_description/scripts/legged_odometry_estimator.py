#!/usr/bin/env python3
"""
腿式机器人里程计估计节点（真机用）
用法：基于足端接触信息 + IMU 数据估计全局位移

主要思路：
1. 监听足部接触传感器和 IMU 数据
2. 通过运动学计算接地足端的位移
3. 融合 IMU 的陀螺仪数据得到方向变化
4. 发布 odom -> base_link 的 TF 变换和 /odom 话题
"""

import rospy
import tf2_ros
import numpy as np
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from std_srvs.srv import Empty
import tf.transformations as tft

class LeggedOdometryEstimator:
    def __init__(self):
        rospy.init_node('legged_odometry_estimator')
        
        # 参数（根据您的 N2 机器人调整）
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        
        # 机器人学参数
        self.leg_names = ['FL_leg', 'FR_leg', 'BL_leg', 'BR_leg']  # 前左、前右、后左、后右
        self.hip_offsets = {
            'FL_leg': np.array([0.15, 0.1, 0.0]),    # 前左髋关节在 base_link 中的位置
            'FR_leg': np.array([0.15, -0.1, 0.0]),   # 前右
            'BL_leg': np.array([-0.15, 0.1, 0.0]),   # 后左
            'BR_leg': np.array([-0.15, -0.1, 0.0]),  # 后右
        }
        self.leg_length = rospy.get_param('~leg_length', 0.4)  # 腿部总长度
        
        # 状态变量
        self.x = 0.0           # 全局 X 位置
        self.y = 0.0           # 全局 Y 位置
        self.theta = 0.0       # 全局 yaw 角
        self.vx = 0.0          # X 方向速度
        self.vy = 0.0          # Y 方向速度
        self.vtheta = 0.0      # 旋转速度
        self.last_time = rospy.Time.now()
        
        # IMU 数据缓存
        self.imu_data = None
        self.joint_states = None
        self.contact_states = None  # 接触信息
        
        # TF 广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 发布器
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
        self.vel_pub = rospy.Publisher('/odom_twist', Twist, queue_size=50)
        
        # 订阅器
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        # 注意：下面这个话题名称需要根据你的足部传感器话题修改
        rospy.Subscriber('/contact_sensors', Imu, self.contact_callback)
        
        # 重置里程计服务
        rospy.Service('~reset_odometry', Empty, self.reset_odometry)
        
        rospy.loginfo("Legged Odometry Estimator initialized")
        
    def imu_callback(self, msg):
        """处理 IMU 数据"""
        self.imu_data = msg
        
    def joint_state_callback(self, msg):
        """处理关节状态（四足机器人有 12 个自由度，通常 3 个/条腿）"""
        self.joint_states = msg
        
    def contact_callback(self, msg):
        """处理接触传感器数据（监听哪条腿接地）"""
        # 注意：这取决于你的足部传感器实际发布的格式
        self.contact_states = msg
        
    def reset_odometry(self, req):
        """重置里程计"""
        rospy.logwarn("Resetting odometry to origin")
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = rospy.Time.now()
        return []
        
    def estimate_swing_leg_displacement(self):
        """
        从关节状态估计摇摆腿（非接地腿）的端点位移
        这是一个简化版本，实际需要完整的运动学链
        """
        if self.joint_states is None:
            return np.array([0.0, 0.0, 0.0])
        
        # TODO: 实现完整的正向运动学
        # 这里需要针对 N2 的具体关节配置
        displacement = np.array([0.0, 0.0, 0.0])
        return displacement
        
    def update_odometry(self):
        """更新里程计估计"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        
        if dt <= 0 or dt > 1.0:
            return
        
        # ============ 从 IMU 获取旋转速度 ============
        if self.imu_data is not None:
            # 陀螺仪的 Z 轴分量（绕竖直轴的旋转）
            self.vtheta = self.imu_data.angular_velocity.z
            
            # 更新 yaw 角
            self.theta += self.vtheta * dt
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # 规范到 [-pi, pi]
        
        # ============ 从足端接触和关节状态估计线速度 ============
        # 这是简化版：实际应该用所有接地腿的平均位移
        if self.joint_states is not None and self.contact_states is not None:
            # 获取摇摆腿的位移
            swing_displacement = self.estimate_swing_leg_displacement()
            
            # 在基座坐标系中的线速度（简化：假设摇摆腿端点位移 ≈ 躯干位移）
            self.vx = swing_displacement[0] / dt if dt > 0 else 0
            self.vy = swing_displacement[1] / dt if dt > 0 else 0
        
        # ============ 更新全局位置（简单积分） ============
        # 在全局坐标系中更新位置
        dx = self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)
        dy = self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)
        
        self.x += dx * dt
        self.y += dy * dt
        
        # 发布里程计信息
        self.publish_odometry(current_time)
        
    def publish_odometry(self, stamp):
        """发布里程计 TF 和消息"""
        # ============ 发布 TF: odom -> base_link ============
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        
        # 从 yaw 角构建四元数
        q = tft.quaternion_from_euler(0, 0, self.theta)
        transform.transform.rotation = Quaternion(*q)
        
        self.tf_broadcaster.sendTransform(transform)
        
        # ============ 发布 /odom 话题 ============
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # 位置
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*tft.quaternion_from_euler(0, 0, self.theta))
        
        # 速度
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vtheta
        
        # 协方差（可调）
        odom.pose.covariance = [0.1, 0, 0, 0, 0, 0,
                                0, 0.1, 0, 0, 0, 0,
                                0, 0, 0.1, 0, 0, 0,
                                0, 0, 0, 0.1, 0, 0,
                                0, 0, 0, 0, 0.1, 0,
                                0, 0, 0, 0, 0, 0.1]
        odom.twist.covariance = odom.pose.covariance
        
        self.odom_pub.publish(odom)
        
        # 发布速度
        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
        twist.angular.z = self.vtheta
        self.vel_pub.publish(twist)
        
    def run(self):
        """主循环"""
        rate = rospy.Rate(50)  # 50 Hz
        while not rospy.is_shutdown():
            self.update_odometry()
            rate.sleep()

if __name__ == '__main__':
    try:
        estimator = LeggedOdometryEstimator()
        estimator.run()
    except rospy.ROSInterruptException:
        pass
