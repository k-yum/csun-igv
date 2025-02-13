#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64
import serial
import time

# Serial port where Arduino is connected (adjust this as necessary)
serial_port = '/dev/ttyACM1'  # For Linux
# serial_port = 'COM3'         # For Windows

# Initialize the serial connection to Arduino
arduino = serial.Serial(serial_port, 9600, timeout=1)

# ROS Publishers
right_encoder_pub = rospy.Publisher('right_encoder_ticks', Int64, queue_size=10)
left_encoder_pub = rospy.Publisher('left_encoder_ticks', Int64, queue_size=10)

def read_encoder_data():
    try:
        # Read the data from the Arduino's serial output
        line = arduino.readline().decode('utf-8').strip()
        if "RightEncoder:" in line and "LeftEncoder:" in line:
            # Parse the data from the string
            parts = line.split(", ")
            right_ticks = int(parts[0].split(": ")[1])
            left_ticks = int(parts[1].split(": ")[1])
            return right_ticks, left_ticks
        else:
            return None, None
    except Exception as e:
        rospy.logwarn(f"Error reading data: {e}")
        return None, None

def encoder_publisher():
    # Initialize the ROS node
    rospy.init_node('encoders')

    rate = rospy.Rate(10)  # 10 Hz loop rate

    while not rospy.is_shutdown():
        right_ticks, left_ticks = read_encoder_data()

        if right_ticks is not None and left_ticks is not None:
            right_msg = Int64()
            left_msg = Int64()

            right_msg.data = right_ticks
            left_msg.data = left_ticks

            # Publish the encoder tick data
            right_encoder_pub.publish(right_msg)
            left_encoder_pub.publish(left_msg)

            rospy.loginfo(f"Right Encoder Ticks: {right_ticks}")
            rospy.loginfo(f"Left Encoder Ticks: {left_ticks}")

        rate.sleep()

if __name__ == '__main__':
    try:
        encoder_publisher()
    except rospy.ROSInterruptException:
        pass
    finally:
        arduino.close()  # Close the serial connection when done

