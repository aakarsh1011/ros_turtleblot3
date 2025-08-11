#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance')

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        self.cmd = Twist()
        self.safe_distance = 0.5  # meters
        self.forward_speed = 0.2
        self.turn_speed = 0.5
        self.angle_range = 30     # Degrees around the front to check

        rospy.loginfo("Obstacle avoidance node started.")
        rospy.spin()

    def laser_callback(self, scan_msg):
        num_ranges = len(scan_msg.ranges)
        center_index = num_ranges // 2
        range_window = scan_msg.ranges[center_index - self.angle_range : center_index + self.angle_range + 1]

        # Filter invalid readings
        valid_ranges = [r for r in range_window if not math.isinf(r) and not math.isnan(r)]

        if not valid_ranges:
            front_distance = scan_msg.range_max
        else:
            front_distance = min(valid_ranges)

        rospy.loginfo("Valid front distances: {}".format([round(r, 2) for r in valid_ranges]))
        rospy.loginfo("Minimum distance ahead: {:.2f} m".format(front_distance))

        if front_distance > self.safe_distance:
            self.cmd.linear.x = self.forward_speed
            self.cmd.angular.z = 0.0
            rospy.loginfo("Path clear. Moving forward.")
        else:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = self.turn_speed
            rospy.logwarn("Obstacle ahead! Turning left.")

        self.cmd_pub.publish(self.cmd)

if __name__ == '__main__':
    try:
        ObstacleAvoidance()
    except rospy.ROSInterruptException:
        pass
