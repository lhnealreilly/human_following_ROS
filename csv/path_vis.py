import csv
import matplotlib.pyplot as plt
import numpy as np

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

des_variance_means = []
human_variance_means = []

for sen in scenarios:
    human_x = []
    human_y = []
    astar_x = []
    astar_y = []
    spring_x = []
    spring_y = []
    with open('results_astar_' + sen + '.csv', newline='') as csvfile:
        for row in csv.reader(csvfile, delimiter=','):
            astar_x.append(list(map(lambda x: float(x), row))[col])
            astar_y.append(list(map(lambda x: float(x), row))[col+1])
            human_x.append(list(map(lambda x: float(x), row))[0])
            human_y.append(list(map(lambda x: float(x), row))[1])     
    with open('results_spring_' + sen + '.csv', newline='') as csvfile:
        for row in csv.reader(csvfile, delimiter=','):
            spring_x.append(list(map(lambda x: float(x), row))[col])
            spring_y.append(list(map(lambda x: float(x), row))[col+1])
    plt.figure()
    for obstacle in obstacles[sen]:
        plt.plot(obstacle[0], obstacle[1], 'black')
    
    plt.title(sen + ' Variance from Desired')
    plt.rcParams["figure.figsize"] = [7.00, 3.50]
    plt.rcParams["figure.autolayout"] = True
    plt.plot(human_x, human_y, label='human')
    plt.plot(astar_x[2:], astar_y[2:], label='astar')
    plt.plot(spring_x[2:], spring_y[2:], label='spring')
    plt.legend()
    plt.savefig(sen + '_path.png')




