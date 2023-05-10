#!/usr/bin/env python3

import math
import numpy as np
from VirtualSpring import VirtualSpring
import matplotlib.pyplot as plt

human = [0.6, -0.81, np.pi/4]
ranges = [-5, 5]

VirtualSpring = VirtualSpring(desired_follow_angle=1*np.pi/4, desired_follow_distance=2, desired_angle_range=[0, 0], desired_distance_range=[0, 0], config_file='config.ini')

            
def update_robot(x, y):
    VirtualSpring.updateRobot([x, y])

def update_human(x, y, theta):
    VirtualSpring.updateHuman([x, y, theta])

def update_walls(Obstacles):
        obstacles_array = []
        for wall in Obstacles:
            obstacles_array.append({'type': 'line', 'position': [[wall[0][0], wall[1][0]], [wall[0][1], wall[1][1]]]})
        VirtualSpring.updateObstacles(obstacles_array)


ob_array = [[[-5, -0.1], [.55, .55]], [[-0.1, -0.1], [.55, 5]]]
update_walls(ob_array)

update_human(*human)
for i in np.arange(*ranges, 0.5):
    for j in np.arange(*ranges, 0.5):
        update_robot(i, j)
        desired_vel = np.array(VirtualSpring.getRobotControlVelocity())/4
        if np.linalg.norm(desired_vel) == 0 :
            continue
        loc = np.array([i, j])
        arrow = plt.arrow(*(loc - 0.5 * desired_vel), *(desired_vel), linewidth=0.1, head_width=0.1)
        plt.gca().add_patch(arrow)  
circle1 = plt.Circle(human[0:2], 0.25,label='Human', color='r')
plt.gca().add_patch(circle1)

circle2 = plt.Circle([human[0] + math.cos(np.pi/4 - human[2]) * 2.1, human[1] + math.sin(np.pi/4 - human[2]) * 2.1], 0.25,label='Desired', color='orange')
plt.gca().add_patch(circle2)

circle2 = plt.Circle([human[0] + math.cos(3*np.pi/4 - human[2]) * 2.1, human[1] + math.sin(3*np.pi/4 - human[2]) * 2.1], 0.25, color='orange')
plt.gca().add_patch(circle2)

for ob in ob_array:
     plt.plot(ob[0], ob[1], label='wall', c='black')

plt.legend(loc='upper left')
plt.savefig('PotentialField.png')
