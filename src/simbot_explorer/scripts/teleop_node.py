#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_robot():
    #initilize the node
    rospy.init_node("teleop_node")

    #publishing velocity commands
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    #set the loop rate
    rate = rospy.Rate(2) ## 2hz

    rospy.loginfo("Teleop node started")

    while not rospy.is_shutdown():
        #create a Twist message
        move_cmd = Twist()
        move_cmd.linear.x = 0.5 # forward speed (m/s)
        move_cmd.angular.z = 0.5 # angular speed (rad/s) # just z axis as 2-D

        pub.publish(move_cmd)
        rate.sleep()

if __name__ == "__main__":
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass