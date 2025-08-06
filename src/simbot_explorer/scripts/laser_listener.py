#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

#callback function
def scan_callback(scan_msg):
    rospy.loginfo("Received laser scan message")
    centre_index = len(scan_msg.ranges) // 2
    distance = scan_msg.ranges[centre_index]

    if distance == float('inf'):
        rospy.loginfo("No obstacle detected")
    else:
        rospy.loginfo("Obstacle detected at distance: %f meters", distance)

    if distance < 0.5:
        rospy.logwarn("Obstacle detected. Stopping robot")


def listener():
    rospy.init_node('laser_listener', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()