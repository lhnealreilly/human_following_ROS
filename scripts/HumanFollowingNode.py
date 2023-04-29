#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Point, PoseStamped, Pose
from nav_msgs.msg import Odometry
from pedsim_msgs.msg import AgentStates, LineObstacles, Waypoints
import math
import numpy as np
import csv
import argparse
import os, rospkg
from VirtualSpring import VirtualSpring
import tf
from AStarOpen import astar

from visualization_msgs.msg import Marker

class HumanFollower:
    def __init__(self, algorithm, scenario):
        self.VirtualSpring = VirtualSpring(desired_follow_angle=5*np.pi/4, desired_follow_distance=2, desired_angle_range=[0, 0], desired_distance_range=[0, 0], config_file='config.ini')
        print(algorithm, scenario)
        self.algorithm = algorithm
        self.scenario = scenario
        
        rospy.init_node('human_follower', anonymous=True)
        
        self.pub = rospy.Publisher('/pedbot/control/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/pedsim_simulator/robot_position', Odometry, self.update_robot)
        rospy.Subscriber('/pedsim_simulator/simulated_agents', AgentStates, self.update_human)
        rospy.Subscriber('/pedsim_simulator/simulated_walls', LineObstacles, self.update_walls)
        rospy.Subscriber('/pedsim_simulator/simulated_waypoints', Waypoints, self.update_waypoints)

        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.robot = None
        self.human = None
        self.obstacles = []
        self.rate = rospy.Rate(10)

        self.human_goal = []

        self.results = []

    def update_waypoints(self, Waypoints):
        waypoint = Waypoints.waypoints[-1]
        self.human_goal = [waypoint.position.x, waypoint.position.y, waypoint.radius]

    def update_walls(self, Message):
        obstacles = []
        for wall in Message.obstacles:
            obstacles.append({'type': 'line', 'position': [[wall.start.x, wall.start.y], [wall.end.x, wall.end.y]]})
        self.obstacles = obstacles
        self.VirtualSpring.updateObstacles(obstacles)
            
    def update_robot(self, Odom):
        self.robot = [Odom.pose.pose.position.x, Odom.pose.pose.position.y]
        self.VirtualSpring.updateRobot([Odom.pose.pose.position.x, Odom.pose.pose.position.y])

    def update_human(self, Humans):
        quat = Humans.agent_states[0].pose.orientation
        heading = -tf.transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))[2] - np.pi/2
        self.human = [Humans.agent_states[0].pose.position.x, Humans.agent_states[0].pose.position.y, heading]
        self.VirtualSpring.updateHuman([Humans.agent_states[0].pose.position.x, Humans.agent_states[0].pose.position.y, heading])

    def run(self):
        start_time = rospy.Time.now()
        rospack = rospkg.RosPack()
        while not rospy.is_shutdown():
            if self.robot is None or self.human is None:
                continue
            cmd_vel = Twist()
            desired_pos = self.VirtualSpring.getDesired()
            if self.algorithm == "spring":
                desired_vel = self.VirtualSpring.getRobotControlVelocity()
                cmd_vel.linear.x = desired_vel[0]
                cmd_vel.linear.y = desired_vel[1]
            elif self.algorithm == "astar":
                path = astar([*self.obstacles, {'type': 'circle', 'position': [self.human[0], self.human[1], 1]}], tuple(self.robot), tuple(desired_pos), limit=90)
                if path != []  and len(path[0]) > 2:
                    if np.linalg.norm(np.array([path[0][2], path[1][2]]) - np.array(self.robot)) < 1:    
                        cmd_vel.linear.x, cmd_vel.linear.y = 1 / np.linalg.norm(np.array([path[0][2], path[1][2]]) - np.array(self.robot)) * (np.array([path[0][2], path[1][2]]) - np.array(self.robot))
                    else:
                        cmd_vel.linear.x, cmd_vel.linear.y = (np.array([path[0][2], path[1][2]]) - np.array(self.robot))
                        
            self.pub.publish(cmd_vel)
            print(self.human)
            self.results.append([*self.human, *self.robot, rospy.Time.now(), math.sqrt((desired_pos[0] - self.robot[0]) **2 + (desired_pos[1] - self.robot[1])**2), math.sqrt((self.human[0] - self.robot[0]) **2 + (self.human[1] - self.robot[1])**2), self.VirtualSpring.getOccluded()])
            if(len(self.human_goal) > 0 and self.VirtualSpring.dist(self.human, self.human_goal) < self.human_goal[2]):
                with open(os.path.join(rospack.get_path("human_following"), "csv", "results_" + self.algorithm + "_" + self.scenario + ".csv"), 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    for state in self.results:
                        writer.writerow(state)
                    exit()
            
            

            #Marker to show the robot's desired position based on follow distance and angle
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.ns = "marked"
            marker.id = 0
            marker.type = marker.POINTS
            marker.action = marker.ADD
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 0.1
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z= 0
            marker.pose.orientation.w = 1
            marker.color.a = 0.4
            marker.color.r = 0.9
            marker.color.g = 0.1
            marker.color.b = 0.2
            marker.lifetime = rospy.Duration()

            point = Point()
            point.x = desired_pos[0]
            point.y = desired_pos[1]
            marker.points = [point]
            
            self.marker_pub.publish(marker)
            self.rate.sleep()
            

if __name__ == '__main__':
    rospack = rospkg.RosPack()

    parser = argparse.ArgumentParser(description='Test path following algorithms.')
    parser.add_argument('--algorithm', type=str, choices=["spring", "astar"], help='which path following algorithms to test ("spring", "astar")')
    parser.add_argument('--scenario', type=str, help='what scenario is being run')
    args, unknown = parser.parse_known_args()

    algorithm = args.algorithm
    scenario = args.scenario
    
    try:
        human_follower = HumanFollower(algorithm, scenario)
        human_follower.run()
    except rospy.ROSInterruptException:
        pass
