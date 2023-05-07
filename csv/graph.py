import csv
import matplotlib.pyplot as plt
import numpy as np
from statistics import mean

scenarios = ['track', 'track_inv', 'corridors', 'corridors_inv', 'doorway', 'zigzag']

col = 5

des_variance_means = []
human_variance_means = []

for sen in scenarios:
    astar_variance_des = []
    astar_variance_human = []
    astar_occlusion = []
    spring_variance_des = []
    spring_variance_human = []
    spring_occlusion = []
    with open('results_astar_' + sen + '.csv', newline='') as csvfile:
        for row in csv.reader(csvfile, delimiter=','):
            astar_variance_des.append(list(map(lambda x: float(x), row))[col])
            astar_variance_human.append(list(map(lambda x: float(x), row))[col+1])
            astar_occlusion.append(list(map(lambda x: float(x), row))[col+2]/10)     
    with open('results_spring_' + sen + '.csv', newline='') as csvfile:
        for row in csv.reader(csvfile, delimiter=','):
            spring_variance_des.append(list(map(lambda x: float(x), row))[col])
            spring_variance_human.append(list(map(lambda x: float(x), row))[col+1])
            spring_occlusion.append(list(map(lambda x: float(x), row))[col+2]/10)

    plt.figure()
    plt.title(sen + ' Variance from Desired')
    plt.rcParams["figure.figsize"] = [7.00, 3.50]
    plt.rcParams["figure.autolayout"] = True
    plt.plot(astar_variance_des, label='astar')
    plt.plot(spring_variance_des, label='spring')
    plt.legend()
    plt.ylabel("Variance")
    plt.xlabel("Time")
    plt.savefig(sen + '_variance_des.png')

    plt.figure()
    plt.title(sen + ' Variance from Human')
    plt.rcParams["figure.figsize"] = [7.00, 3.50]
    plt.rcParams["figure.autolayout"] = True
    plt.plot(astar_variance_human, label='astar')
    plt.plot(spring_variance_human, label='spring')
    plt.legend()
    plt.ylabel("Variance")
    plt.xlabel("Time")
    plt.savefig(sen + '_variance_human.png')

    plt.figure()
    plt.title(sen + ' Occlusion')
    plt.rcParams["figure.figsize"] = [7.00, 3.50]
    plt.rcParams["figure.autolayout"] = True
    plt.plot(astar_occlusion, label='astar')
    plt.plot(spring_occlusion, label='spring')
    plt.legend()
    plt.ylabel("Occlusion")
    plt.xlabel("Algorithm")
    plt.savefig(sen + '_occlusion.png')
    print(sen, mean(astar_variance_human), mean(spring_variance_human))
    des_variance_means.append([[mean(astar_variance_des), sen + ' AStar'], [mean(spring_variance_des), sen + ' Spring']])
    human_variance_means.append([[mean(astar_variance_human), sen + ' AStar'], [mean(spring_variance_human), sen + ' Spring']])

xticks = []
plt.figure()
plt.title('Average Variance from Desired Location')
plt.rcParams["figure.figsize"] = [7.00, 3.50]
plt.rcParams["figure.autolayout"] = True
for i, mean in enumerate(des_variance_means):
    plt.bar(i*2, mean[0][0], width=-0.8, align='edge', color='b')
    plt.bar(i*2, mean[1][0], width=0.8, align='edge', color='orange')
    xticks.extend([mean[0][1], mean[1][1]])
plt.ylabel("Mean")
plt.xlabel("Scenario")
plt.xticks(np.arange(-0.5, len(xticks), 1), xticks, rotation=45, ha="right")
plt.savefig('des_means.png')

xticks = []
plt.figure()
plt.title('Average Distance from Human Location')
plt.rcParams["figure.figsize"] = [7.00, 3.50]
plt.rcParams["figure.autolayout"] = True
for i, mean in enumerate(human_variance_means):
    plt.bar(i*2, mean[0][0], width=-0.8, align='edge', color='b')
    plt.bar(i*2, mean[1][0], width=0.8, align='edge', color='orange')
    xticks.extend([mean[0][1], mean[1][1]])
plt.axhline(y=2, color='r', linestyle='-', label='Desired')
plt.ylabel("Mean")
plt.xlabel("Scenario")
plt.legend(loc='upper right', bbox_to_anchor=(1, 1.3))
plt.xticks(np.arange(-0.5, len(xticks), 1), xticks, rotation=45, ha="right")
plt.savefig('human_means.png', bbox_inches='tight')



