#!/usr/bin/env python3

import rospy
import serial
import math
import tf
from tf.transformations import quaternion_from_euler

# Encoder and robot parameters
WHEEL_RADIUS = 0.19  # meters (adjust according to your robot's wheel size)
BASE_WIDTH = 0.762    # meters (adjust according to your robot's base width)
ENCODER_PPR = 1024   # Pulses per revolution for US Digital encoder

# Set up serial communication (adjust the port as per your system)
SERIAL_PORT = '/dev/ttyACM1'  # Adjust the port as per your system
BAUD_RATE = 115200

# Global variables to store encoder counts and odometry data
left_encoder_ticks = 0
right_encoder_ticks = 0
last_left_ticks = 0
last_right_ticks = 0
robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0

# Initialize the time variable
last_time = None  # We'll set this after the node is initialized

def encoder_callback():
    global left_encoder_ticks, right_encoder_ticks, last_left_ticks, last_right_ticks
    global robot_x, robot_y, robot_theta, last_time

    # Open serial port to read from Arduino
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        while not rospy.is_shutdown():
            try:
                # Read raw data from the serial port
                line = ser.readline()  # This reads the data as raw bytes
                if line:
                    line_str = line.decode('utf-8').strip()  # Decode the line to a string
                    rospy.loginfo(f"Received data: {line_str}")

                    # Assuming Arduino sends left and right encoder counts as "L:left_ticks R:right_ticks"
                    data = line_str.split()
                    if len(data) == 2:
                        left_ticks = int(data[0].split(":")[1])
                        right_ticks = int(data[1].split(":")[1])

                        # Compute the change in encoder ticks
                        delta_left_ticks = left_ticks - last_left_ticks
                        delta_right_ticks = right_ticks - last_right_ticks

                        # Update last encoder ticks
                        last_left_ticks = left_ticks
                        last_right_ticks = right_ticks

                        # Compute linear and angular distances based on encoder counts
                        left_distance = (delta_left_ticks / ENCODER_PPR) * (2 * math.pi * WHEEL_RADIUS)
                        right_distance = (delta_right_ticks / ENCODER_PPR) * (2 * math.pi * WHEEL_RADIUS)

                        # Compute robot's change in position
                        delta_distance = (left_distance + right_distance) / 2.0
                        delta_theta = (right_distance - left_distance) / BASE_WIDTH

                        # Update robot's position
                        robot_x += delta_distance * math.cos(robot_theta)
                        robot_y += delta_distance * math.sin(robot_theta)
                        robot_theta += delta_theta

                        # Publish TF transform for the robot's position
                        br.sendTransform((robot_x, robot_y, 0),  # Robot position in x, y, z
                                         quaternion_from_euler(0, 0, robot_theta),  # Robot's orientation as quaternion
                                         rospy.Time.now(),  # Current time
                                         "base_link",  # The frame name for the robot base
                                         "odom")  # The reference frame (e.g., odom frame)

            except Exception as e:
                rospy.logwarn(f"Error reading serial data: {e}")
                rospy.sleep(0.1)

def encoder_node():
    # Initialize the ROS node
    rospy.init_node('encoder_node', anonymous=True)

    # Create a TF broadcaster
    global br
    br = tf.TransformBroadcaster()

    # Start the encoder callback to read data from Arduino
    encoder_callback()

if __name__ == "__main__":
    try:
        encoder_node()
    except rospy.ROSInterruptException:
        pass

