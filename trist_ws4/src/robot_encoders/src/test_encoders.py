#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import serial

def encoder_test_node():
    rospy.init_node('encoder_test_node', anonymous=True)
    left_pub = rospy.Publisher('/left_encoder', Float64, queue_size=10)
    right_pub = rospy.Publisher('/right_encoder', Float64, queue_size=10)

    ser = serial.Serial('/dev/ttyACM1', 200000)  # Adjust to your Arduino's port

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            left_pos, right_pos = map(float, line.split(','))

            # Publish encoder values
            left_pub.publish(left_pos)
            right_pub.publish(right_pos)

            rospy.loginfo("Left Encoder: %s, Right Encoder: %s", left_pos, right_pos)

        rate.sleep()

if __name__ == '__main__':
    try:
        encoder_test_node()
    except rospy.ROSInterruptException:
        pass

