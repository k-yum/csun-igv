#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

def gpsCallback(msg):
    pose = PoseStamped()
    pose.pose.position.y = msg.latitude
    pose.pose.position.x = msg.longitude
    local_origins_pub.publish(pose)
    origin_gps_initialized = True

if __name__ == "__main__":
    rospy.init_node("Initialize_origin_GPS")
    rospy.Subscriber("/gps_data", NavSatFix, gpsCallback)
    local_origins_pub = rospy.Publisher("/local_xy_origin", PoseStamped, queue_size=10)
    r = rospy.Rate(10)

    origin_gps_initialized = False
    while not rospy.is_shutdown():
        if origin_gps_initialized:
            rospy.loginfo("Initialized GPS for the origin.")
            break
        r.sleep()
