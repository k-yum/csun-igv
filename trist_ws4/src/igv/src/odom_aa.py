#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

# Create odometry data publishers
odom_data_pub = None
odom_data_pub_quat = None
odomNew = Odometry()
odomOld = Odometry()

# Initial pose
initialX = 0.0
initialY = 0.0
initialTheta = 1e-10
PI = 3.141592

# Robot physical constants
TICKS_PER_REVOLUTION = 32768
WHEEL_RADIUS = 0.1905  # meters
WHEEL_BASE = 0.7747  # meters
TICKS_PER_METER = 27376.32

# Distance both wheels have traveled
distanceLeft = 0
distanceRight = 0

# Flag to see if initial pose has been received
initialPoseReceived = False

# Get initial_2d message from either RViz clicks or a manual pose publisher
def set_initial_2d(rvizClick):
    global odomOld, initialPoseReceived
    odomOld.pose.pose.position.x = rvizClick.pose.position.x
    odomOld.pose.pose.position.y = rvizClick.pose.position.y
    odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z
    initialPoseReceived = True

# Calculate the distance the left wheel has traveled since the last cycle
def calc_left(leftCount):
    global distanceLeft, lastCountL
    if leftCount.data != 0 and lastCountL != 0:
        leftTicks = leftCount.data - lastCountL
        if leftTicks > 10000:
            leftTicks = 0 - (65535 - leftTicks)
        elif leftTicks < -10000:
            leftTicks = 65535 - leftTicks
        distanceLeft = leftTicks / TICKS_PER_METER
    lastCountL = leftCount.data

# Calculate the distance the right wheel has traveled since the last cycle
def calc_right(rightCount):
    global distanceRight, lastCountR
    if rightCount.data != 0 and lastCountR != 0:
        rightTicks = rightCount.data - lastCountR
        if rightTicks > 10000:
            distanceRight = (0 - (65535 - distanceRight)) / TICKS_PER_METER
        elif rightTicks < -10000:
            rightTicks = 65535 - rightTicks
        distanceRight = rightTicks / TICKS_PER_METER
    lastCountR = rightCount.data

# Publish a nav_msgs::Odometry message in quaternion format
def publish_quat():
    global odomNew, odom_data_pub_quat
    q = quaternion_from_euler(0, 0, odomNew.pose.pose.orientation.z)
    
    quatOdom = Odometry()
    quatOdom.header.stamp = odomNew.header.stamp
    quatOdom.header.frame_id = "odom"
    quatOdom.child_frame_id = "base_link"
    quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x
    quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y
    quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z
    quatOdom.pose.pose.orientation.x = q[0]
    quatOdom.pose.pose.orientation.y = q[1]
    quatOdom.pose.pose.orientation.z = q[2]
    quatOdom.pose.pose.orientation.w = q[3]
    quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x
    quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y
    quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z
    quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x
    quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y
    quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z

    # Set covariance values
    for i in range(36):
        if i == 0 or i == 7 or i == 14:
            quatOdom.pose.covariance[i] = 0.01
        elif i == 21 or i == 28 or i == 35:
            quatOdom.pose.covariance[i] += 0.1
        else:
            quatOdom.pose.covariance[i] = 0

    # Publish the quaternion odometry
    odom_data_pub_quat.publish(quatOdom)

    # Log that we're publishing
    rospy.loginfo("Publishing Odometry with Quaternion: %s", quatOdom)

# Update odometry information
def update_odom():
    global odomNew, odomOld, distanceLeft, distanceRight

    # Calculate the average distance
    cycleDistance = (distanceRight + distanceLeft) / 2

    # Calculate the number of radians the robot has turned since the last cycle
    cycleAngle = math.asin((distanceRight - distanceLeft) / WHEEL_BASE)

    # Average angle during the last cycle
    avgAngle = cycleAngle / 2 + odomOld.pose.pose.orientation.z

    if avgAngle > PI:
        avgAngle -= 2 * PI
    elif avgAngle < -PI:
        avgAngle += 2 * PI

    # Calculate the new pose (x, y, and theta)
    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + math.cos(avgAngle) * cycleDistance
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + math.sin(avgAngle) * cycleDistance
    odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z

    # Prevent lockup from a single bad cycle
    if math.isnan(odomNew.pose.pose.position.x) or math.isnan(odomNew.pose.pose.position.y):
        odomNew.pose.pose.position.x = odomOld.pose.pose.position.x
        odomNew.pose.pose.position.y = odomOld.pose.pose.position.y
        odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z

    # Make sure theta stays in the correct range
    if odomNew.pose.pose.orientation.z > PI:
        odomNew.pose.pose.orientation.z -= 2 * PI
    elif odomNew.pose.pose.orientation.z < -PI:
        odomNew.pose.pose.orientation.z += 2 * PI

    # Compute the velocity
    current_time = rospy.Time.now()
    odomNew.header.stamp = current_time
    odomNew.twist.twist.linear.x = cycleDistance / (current_time.to_sec() - odomOld.header.stamp.to_sec())
    odomNew.twist.twist.angular.z = cycleAngle / (current_time.to_sec() - odomOld.header.stamp.to_sec())

    # Save the pose data for the next cycle
    odomOld.pose.pose.position.x = odomNew.pose.pose.position.x
    odomOld.pose.pose.position.y = odomNew.pose.pose.position.y
    odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z
    odomOld.header.stamp = current_time

    # Publish the odometry message
    odom_data_pub.publish(odomNew)

# Main function
def main():
    global odom_data_pub, odom_data_pub_quat

    # Initialize the ROS node
    rospy.init_node('ekf_odom_pub')

    # Set the initial pose
    odomNew.header.frame_id = "odom"
    odomNew.pose.pose.position.z = 0
    odomNew.pose.pose.orientation.x = 0
    odomNew.pose.pose.orientation.y = 0
    odomNew.twist.twist.linear.x = 0
    odomNew.twist.twist.linear.y = 0
    odomNew.twist.twist.linear.z = 0
    odomNew.twist.twist.angular.x = 0
    odomNew.twist.twist.angular.y = 0
    odomNew.twist.twist.angular.z = 0
    odomOld.pose.pose.position.x = initialX
    odomOld.pose.pose.position.y = initialY
    odomOld.pose.pose.orientation.z = initialTheta

    # Create publishers
    odom_data_pub = rospy.Publisher("odom_data_euler", Odometry, queue_size=100)
    odom_data_pub_quat = rospy.Publisher("odom_data_quat", Odometry, queue_size=100)

    # Log that the publisher has been initialized
    rospy.loginfo("Publisher for /odom_data_quat initialized")

    # Subscribe to topics
    rospy.Subscriber("right_ticks", Int16, calc_right)
    rospy.Subscriber("left_ticks", Int16, calc_left)
    rospy.Subscriber("initial_2d", PoseStamped, set_initial_2d)

    # Set the loop rate
    rate = rospy.Rate(30)

    # Main loop
    while not rospy.is_shutdown():
        if initialPoseReceived:
            update_odom()
            publish_quat()
        rate.sleep()

if __name__ == '__main__':
    main()

