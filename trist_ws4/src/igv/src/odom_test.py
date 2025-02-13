#!/usr/bin/env python3

# THIS ONE WORKS BUT IT POSITION IS OFF

import rospy
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from math import sin, cos, pi
import tf2_ros
from geometry_msgs.msg import TransformStamped

class OdometryCalculator:
    def __init__(self):
        rospy.init_node('encoder_node')
        self.encoder_sub_left = rospy.Subscriber('/left_encoder_ticks', Int64, self.left_encoder_callback)
        self.encoder_sub_right = rospy.Subscriber('/right_encoder_ticks', Int64, self.right_encoder_callback)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
       
        # Initialize previous tick counts
        self.previous_left_ticks = 0
        self.previous_right_ticks = 0
       
        # Initialize displacement values to zero (for both wheels)
        self.left_displacement = 0.0
        self.right_displacement = 0.0
       
        # Initialize position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
       
        self.last_time = rospy.Time.now()

        # Parameters for wheel odometry (example values, adjust based on your robot)
        self.wheel_radius = 0.1905  # meters
        self.encoder_ticks_per_revolution = 98304  # adjust to your encoder's resolution
        self.wheelbase = 0.7747  # meters, distance between left and right wheels

        # TF broadcasters for transforming between frames
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
       
        # Publish static transform from base_link to base_footprint
        self.publish_base_footprint_to_base_link()

    def left_encoder_callback(self, msg):
        left_ticks = msg.data
        left_delta_ticks = left_ticks - self.previous_left_ticks
        self.previous_left_ticks = left_ticks
       
        # Calculate left wheel displacement (meters)
        self.left_displacement = (left_delta_ticks / self.encoder_ticks_per_revolution) * 2 * pi * self.wheel_radius

    def right_encoder_callback(self, msg):
        right_ticks = msg.data
        right_delta_ticks = right_ticks - self.previous_right_ticks
        self.previous_right_ticks = right_ticks
       
        # Calculate right wheel displacement (meters)
        self.right_displacement = (right_delta_ticks / self.encoder_ticks_per_revolution) * 2 * pi * self.wheel_radius

    def calculate_odometry(self):
        # Get current time and calculate delta time
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
       
        # Modify the displacement calculation to reflect the asymmetry in encoder ticks
        adjusted_left_displacement = -self.left_displacement  # Invert left displacement since ticks are decreasing
        adjusted_right_displacement = self.right_displacement  # Keep right displacement as is since ticks are increasing

        # Calculate change in orientation (theta) from both wheels' displacement
        delta_theta = (adjusted_right_displacement - adjusted_left_displacement) / self.wheelbase
       
        # Update theta (orientation) in robot frame
        self.theta += delta_theta
       
        # Calculate displacement in x and y
        delta_x = (adjusted_left_displacement + adjusted_right_displacement) / 2.0
        delta_y = 0.0  # For differential drive, the robot moves in a straight line (no lateral movement)

        # Update the position of the robot (x, y)
        self.x += delta_x * cos(self.theta)
        self.y += delta_x * sin(self.theta)

        # Publish transform from odom to base_footprint
        self.publish_odometry_transform(current_time)

        # Create and populate Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"  # Changed this to "base_footprint"
       
        # Set position (x, y)
        odom_msg.pose.pose.position = Point(self.x, self.y, 0.0)
       
        # Set orientation (quaternion from theta)
        q = self.euler_to_quaternion(self.theta)
        odom_msg.pose.pose.orientation = Quaternion(*q)
       
        # Set linear and angular velocity (assuming constant velocity)
        odom_msg.twist.twist.linear.x = delta_x / dt
        odom_msg.twist.twist.angular.z = delta_theta / dt

        # Publish odometry message
        self.odom_pub.publish(odom_msg)

    def euler_to_quaternion(self, theta):
        """Convert Euler angle (theta) to quaternion"""
        qw = cos(theta / 2.0)
        qx = 0.0
        qy = 0.0
        qz = sin(theta / 2.0)
        return [qw, qx, qy, qz]

    def publish_base_footprint_to_base_link(self):
        """Publish a static transform between base_link and base_footprint"""
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.child_frame_id = "base_footprint"
       
        t.transform.translation.x = 0.0  # No offset between base_link and base_footprint
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
       
        # No rotation between base_link and base_footprint
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
       
        # Broadcast the static transform
        self.tf_broadcaster.sendTransform(t)

    def publish_odometry_transform(self, current_time):
        """Publish a dynamic transform between odom and base_footprint"""
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
       
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0  # Assuming 2D movement

        # Set the orientation from the robot's theta
        q = self.euler_to_quaternion(self.theta)
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]

        # Broadcast the odometry transform
        self.tf_broadcaster.sendTransform(t)

    def run(self):
        rate = rospy.Rate(30)  # 30 Hz
        while not rospy.is_shutdown():
            self.calculate_odometry()
            rate.sleep()


if __name__ == '__main__':
    try:
        encoder_node = OdometryCalculator()
        encoder_node.run()
    except rospy.ROSInterruptException:
        pass


