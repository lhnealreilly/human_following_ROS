#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
import csv
import argparse
import os, rospkg
from VirtualSpring import VirtualSpring 

class HumanFollower:
    def __init__(self):
        self.VirtualSpring = VirtualSpring(desired_follow_angle=3*np.pi, desired_follow_distance=2)
        rospy.init_node('human_follower', anonymous=True)
        
        self.pub = rospy.Publisher('/pedbot/control/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/pedsim_simulator/robot_position', Odometry, self.update_robot)

        self.lidar = None
        self.dist_to_wall = 0
        self.state = 0
        self.angle = 0
        self.rate = rospy.Rate(10)

    def update_robot(Odom):
        print(Odom)
        self.VirtualSpring.updateRobot([Odom.pose.pose.position.x, Odom.pose.pose.position.y])

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    try:
        human_follower = HumanFollower()
        human_follower.run()
    except rospy.ROSInterruptException:
        pass