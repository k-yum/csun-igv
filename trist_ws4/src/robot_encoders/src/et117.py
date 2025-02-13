#!/usr/bin/env python3

import rospy
import tf
import geometry_msgs.msg
from nav_msgs.msg import Odometry
import math
import serial
import time

def odometry_publisher():
    # Initialize ROS Node
    rospy.init_node('encoder_node')

    # Publisher for odometry
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)

    # Create tf broadcaster
    odom_broadcaster = tf.TransformBroadcaster()

    # Initialize odometry state
    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0.1
    vy = -0.1
    vth = 0.1

    last_time = rospy.Time.now()
    
    ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
    time.sleep(2)  # Allow the Arduino to initialize

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()

        # Compute odometry (basic motion model)
        delta_x = (vx * math.cos(th) - vy * math.sin(th)) * dt
        delta_y = (vx * math.sin(th) + vy * math.cos(th)) * dt
        delta_th = vth * dt

        x += delta_x
        y += delta_y
        th += delta_th

        # Create quaternion for the orientation
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # Publish the transform (using odom and base_link frames)
        odom_broadcaster.sendTransform(
            (x, y, 0.0),  # Translation (x, y, z)
            odom_quat,    # Rotation (quaternion)
            current_time,  # Time
            "base_link",   # Child frame
            "odom"         # Parent frame
        )

        # Create and publish the Odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # Set the position (x, y, z)
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        # Set the velocity (vx, vy, vth)
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        # Publish the odometry message
        odom_pub.publish(odom)

        # Update last_time
        last_time = current_time

        rate.sleep()

if __name__ == '__main__':
    try:
        odometry_publisher()
    except rospy.ROSInterruptException:
        pass

