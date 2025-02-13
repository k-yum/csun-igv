#!/usr/bin/env python3

#THIS ONE WORKS FOR SURE

import rospy
from std_msgs.msg import Int64
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import tf2_ros
import math

class OdometryPublisher:
    def __init__(self):
        rospy.init_node('encoder_node')

        # Parameters (adjust based on your robot)
        self.wheel_radius = 0.1905  # meters
        self.wheelbase = 0.7747  # meters (distance between wheels)
        self.encoder_ticks_per_revolution = 32768  # ticks/rev

        # Initialize variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.previous_left_ticks = 0
        self.previous_right_ticks = 0
        self.last_time = rospy.Time.now()

        # ROS Publishers and Subscribers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
        self.odom_broadcaster = tf2_ros.TransformBroadcaster()
        self.left_encoder_sub = rospy.Subscriber('/left_encoder_ticks', Int64, self.left_encoder_callback)
        self.right_encoder_sub = rospy.Subscriber('/right_encoder_ticks', Int64, self.right_encoder_callback)

        self.current_left_ticks = 0
        self.current_right_ticks = 0

    def left_encoder_callback(self, msg):
        self.current_left_ticks = msg.data

    def right_encoder_callback(self, msg):
        self.current_right_ticks = msg.data

    def calculate_odometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Calculate displacement for each wheel
        left_delta_ticks = -1*(self.current_left_ticks - self.previous_left_ticks) #left encoder ticks decrease as robot moves forward
        right_delta_ticks = self.current_right_ticks - self.previous_right_ticks

        self.previous_left_ticks = self.current_left_ticks
        self.previous_right_ticks = self.current_right_ticks

        left_distance = (left_delta_ticks / self.encoder_ticks_per_revolution) * (2 * math.pi * self.wheel_radius)
        right_distance = (right_delta_ticks / self.encoder_ticks_per_revolution) * (2 * math.pi * self.wheel_radius)

        # Calculate robot linear and angular velocities
        linear_velocity = (left_distance + right_distance) / (2 * dt)
        angular_velocity = (right_distance - left_distance) / (self.wheelbase * dt)

        self.vx = linear_velocity
        self.vth = angular_velocity

        # Update position and orientation
        delta_x = linear_velocity * math.cos(self.th) * dt
        delta_y = linear_velocity * math.sin(self.th) * dt
        delta_th = angular_velocity * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Update last time
        self.last_time = current_time

    def publish_odometry(self):
        # Create and populate Odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Set orientation
        quat = quaternion_from_euler(0, 0, self.th)
        odom.pose.pose.orientation = Quaternion(*quat)

        # Set velocities
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth

        # Publish odometry message
        self.odom_pub.publish(odom)

        # Publish the transform over tf
        odom_trans = TransformStamped()
        odom_trans.header.stamp = rospy.Time.now()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"

        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation.x = quat[0]
        odom_trans.transform.rotation.y = quat[1]
        odom_trans.transform.rotation.z = quat[2]
        odom_trans.transform.rotation.w = quat[3]

        self.odom_broadcaster.sendTransform(odom_trans)

    def run(self):
        rate = rospy.Rate(30)  # 30 Hz
        while not rospy.is_shutdown():
            self.calculate_odometry()
            self.publish_odometry()
            rate.sleep()

if __name__ == '__main__':
    try:
        odometry_publisher = OdometryPublisher()
        odometry_publisher.run()
    except rospy.ROSInterruptException:
        pass

