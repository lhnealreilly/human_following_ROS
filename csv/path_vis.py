import csv
import matplotlib.pyplot as plt
import numpy as np
import math
from statistics import mean

scenarios = ['track', 'track_inv', 'corridors', 'corridors_inv', 'doorway', 'zigzag']

obstacles = {
    'track': [],
    'track_inv': [],
    'corridors': [[[-5, 35],[0, 0]], [[0, 30],[5, 5]], [[0, 30],[25, 25]], [[-5, 35],[30, 30]],
                  [[-5, -5],[0, 30]], [[0, 0],[5, 25]], [[35, 35],[0, 30]], [[30, 30],[5, 25]]],
    'corridors_inv': [[[-5, 35],[0, 0]], [[0, 30],[5, 5]], [[0, 30],[25, 25]], [[-5, 35],[30, 30]],
                  [[-5, -5],[0, 30]], [[0, 0],[5, 25]], [[35, 35],[0, 30]], [[30, 30],[5, 25]]],
    'doorway': [[[-.5, 29.5],[-0.5, -.5]], [[-0.5, -0.5],[-.5, 14.5]], [[-0.5, 29.5],[14.5, 14.5]], [[29.5, 29.5],[-.5, 14.5]],
                  [[14.5, 14.5],[-0.5, 5.5]], [[14.5, 14.5],[7.5, 14.5]]],
    'zigzag': [[[0, 20],[0, 0]], [[0, 15],[5, 5]], [[20, 20],[0, 10]], [[15, 15],[5, 15]],
                  [[20, 25],[10, 10]], [[15, 25],[15, 15]]]
}

col = 2

des_variance_means = 0
human_variance_means = 0

def generate_velocity_array(trajectory):
    velocity_array = []
    for i in range(1, len(trajectory)):
        (x0, y0), t0 = trajectory[i-1]
        (x1, y1), t1  = trajectory[i]
        dt = t1 - t0
        dx = x1 - x0
        dy = y1 - y0
        v = math.sqrt(dx**2 + dy**2)/dt
        velocity_array.append(v  * 10**9)
    return velocity_array

def generate_acceleration_array(trajectory):
    acceleration_array = []
    for i in range(1, len(trajectory)-1):
        (x0, y0) , t0 = trajectory[i-1]
        (x1, y1), t1 = trajectory[i]
        (x2, y2), t2  = trajectory[i+1]
        dt1 = t1 - t0
        dt2 = t2 - t1
        dx1 = x1 - x0
        dy1 = y1 - y0
        dx2 = x2 - x1
        dy2 = y2 - y1
        v1x = dx1/dt1
        v1y = dy1/dt1
        v2x = dx2/dt2
        v2y = dy2/dt2
        a = ((v2x - v1x)/dt1 + (v2x - v1x)/dt2)/2
        acceleration_array.append(a  * 10**9)
    return acceleration_array

def generate_jerk_array(trajectory):
    print(trajectory)
    jerk_array = []
    for i in range(2, len(trajectory)-2):
        (x0, y0), t0  = trajectory[i-2]
        (x1, y1), t1  = trajectory[i-1]
        (x2, y2), t2 = trajectory[i]
        (x3, y3), t3 = trajectory[i+1]
        (x4, y4), t4  = trajectory[i+2]

        # Calculate velocity and acceleration vectors
        dt1 = t2 - t1
        dt2 = t3 - t2
        dt3 = t4 - t3
        dx1 = x2 - x1
        dy1 = y2 - y1
        dx2 = x3 - x2
        dy2 = y3 - y2
        dx3 = x4 - x3
        dy3 = y4 - y3
        v2x = dx1/dt1
        v2y = dy1/dt1
        v3x = dx2/dt2
        v3y = dy2/dt2
        v4x = dx3/dt3
        v4y = dy3/dt3

        # Calculate acceleration vectors
        a2x = (v3x - v2x)/((dt1 + dt2)/2)
        a2y = (v3y - v2y)/((dt1 + dt2)/2)
        a3x = (v4x - v3x)/((dt2 + dt3)/2)
        a3y = (v4y - v3y)/((dt2 + dt3)/2)

        # Calculate jerk vector
        dt = (dt1 + dt2 + dt3)/3
        jx = (a3x - a2x)/dt
        jy = (a3y - a2y)/dt
        j = math.sqrt(jx**2 + jy**2)
        jerk_array.append(j * 10**9)
    return jerk_array

jerk_means = []

for sen in scenarios:
    human_x = []
    human_y = []
    astar_x = []
    astar_y = []
    astar_time = []
    spring_x = []
    spring_y = []
    spring_time = []
    with open('results_astar_' + sen + '.csv', newline='') as csvfile:
        for row in csv.reader(csvfile, delimiter=','):
            astar_x.append(list(map(lambda x: float(x), row))[col])
            astar_y.append(list(map(lambda x: float(x), row))[col+1])
            astar_time.append(list(map(lambda x: float(x), row))[col+2])
            human_x.append(list(map(lambda x: float(x), row))[0])
            human_y.append(list(map(lambda x: float(x), row))[1])     
    with open('results_spring_' + sen + '.csv', newline='') as csvfile:
        for row in csv.reader(csvfile, delimiter=','):
            spring_x.append(list(map(lambda x: float(x), row))[col])
            spring_y.append(list(map(lambda x: float(x), row))[col+1])
            spring_time.append(list(map(lambda x: float(x), row))[col+2])
    plt.figure()
    for obstacle in obstacles[sen]:
        plt.plot(obstacle[0], obstacle[1], 'black')

    # astar_vel = []
    # spring_vel = []
    # for i in range(len(astar_x)-1):
    #     astar_vel.append([(astar_x[i] - astar_x[i+1])  / (astar_time[i+1] - astar_time[i]), (astar_y[i] - astar_y[i+1]) / (astar_time[i+1] - astar_time[i])])
    
    # for i in range(len(spring_x)-1):
    #     spring_vel.append([(spring_x[i] - spring_x[i+1])  / (spring_time[i+1] - spring_time[i]), (spring_y[i] - spring_y[i+1]) / (spring_time[i+1] - spring_time[i])])
    # astar_accel = []
    # spring_accel = []
    # for i in range(len(astar_vel)-1):
    #     astar_accel.append([(astar_vel[i][0] - astar_vel[i+1][0]) * 10**9, (astar_vel[i][1] - astar_vel[i+1][1]) * 10**9])
    
    # for i in range(len(spring_vel)-1):
    #     spring_accel.append([(spring_vel[i][0] - spring_vel[i+1][0]) * 10**9, (spring_vel[i][1] - spring_vel[i+1][1]) * 10**9])
    

    astar_vel = generate_velocity_array(list(zip(zip(astar_x, astar_y), astar_time)))
    spring_vel  = generate_velocity_array(list(zip(zip(spring_x, spring_y), spring_time)))

    astar_accel = generate_velocity_array(list(zip(zip(astar_x, astar_y), astar_time)))
    spring_accel  = generate_velocity_array(list(zip(zip(spring_x, spring_y), spring_time)))

    # astar_jerk = generate_jerk_array(list(zip(zip(astar_x, astar_y), astar_time)))
    # spring_jerk = generate_jerk_array(list(zip(zip(spring_x, spring_y), spring_time)))
    astar_jerk = []
    spring_jerk = []
    for i in range(len(astar_accel)-1):
        astar_jerk.append(abs(astar_accel[i] - astar_accel[i+1]))
    
    for i in range(len(spring_accel)-1):
        spring_jerk.append(abs(spring_accel[i] - spring_accel[i+1]))

    
    plt.figure()
    plt.title(sen + ' Jerk')
    plt.rcParams["figure.figsize"] = [7.00, 3.50]
    plt.rcParams["figure.autolayout"] = True
    plt.plot(range(len(astar_jerk)), astar_jerk, label='astar', color='blue')
    plt.plot(range(len(spring_jerk)), spring_jerk, label='spring', color='orange')
    plt.legend()
    plt.savefig(sen + '_jerk.png')

    plt.figure()
    plt.title(sen + ' Velocity')
    plt.rcParams["figure.figsize"] = [7.00, 3.50]
    plt.rcParams["figure.autolayout"] = True
    plt.plot(range(len(astar_vel)), astar_vel, label='astar', color='blue')
    plt.plot(range(len(spring_vel)), spring_vel, label='spring', color='orange')
    plt.legend()
    plt.savefig(sen + '_vel.png')

    plt.figure()
    plt.title(sen + ' Acceleration')
    plt.rcParams["figure.figsize"] = [7.00, 3.50]
    plt.rcParams["figure.autolayout"] = True
    plt.plot(range(len(astar_accel)), astar_accel, label='astar', color='blue')
    plt.plot(range(len(spring_accel)), spring_accel, label='spring', color='orange')
    plt.legend()
    plt.savefig(sen + '_accel.png')

    plt.figure()
    plt.title(sen + ' Path')
    plt.rcParams["figure.figsize"] = [7.00, 3.50]
    plt.rcParams["figure.autolayout"] = True
    plt.plot(human_x, human_y, label='human', color='red')
    plt.plot(astar_x[2:], astar_y[2:], label='astar', color='blue')
    plt.plot(spring_x[2:], spring_y[2:], label='spring', color='orange')
    plt.legend()
    plt.savefig(sen + '_path.png')

    jerk_means.append([[mean(astar_jerk), sen + ' AStar'], [mean(spring_jerk), sen + ' Spring']])

xticks = []
plt.figure()
plt.title('Average Jerk')
plt.rcParams["figure.figsize"] = [7.00, 3.50]
plt.rcParams["figure.autolayout"] = True
for i, mean in enumerate(jerk_means):
    plt.bar(i*2, mean[0][0], width=-0.8, align='edge', color='b')
    plt.bar(i*2, mean[1][0], width=0.8, align='edge', color='orange')
    xticks.extend([mean[0][1], mean[1][1]])
plt.ylabel("Mean")
plt.xlabel("Scenario")
plt.xticks(np.arange(-0.5, len(xticks)-1, 1), xticks, rotation=45, ha="right")
plt.savefig('avg_jerks.png')


