import csv
import matplotlib.pyplot as plt
import numpy as np
from statistics import mean

scenarios = ['track', 'track_inv', 'corridors', 'corridors_inv', 'doorway', 'zigzag']

col = 5

des_variance_means = []
human_variance_means = []
occlusions = []

current_results = []

with open('results.csv', newline='') as csvfile:
    for i, row in enumerate(csv.reader(csvfile, delimiter=',')):
        current_results.append(np.array([float(x) for x in row]) if i > 1 else [x for x in row])

for sen in scenarios:
    astar_variance_des = []
    astar_variance_human = []
    astar_occlusion = 0
    spring_variance_des = []
    spring_variance_human = []
    spring_occlusion = 0
    with open('results_astar_' + sen + '.csv', newline='') as csvfile:
        for row in csv.reader(csvfile, delimiter=','):
            astar_variance_des.append(list(map(lambda x: float(x), row))[col])
            astar_variance_human.append(list(map(lambda x: float(x), row))[col+1])
            astar_occlusion += list(map(lambda x: float(x), row))[col+2]     
    with open('results_spring_' + sen + '.csv', newline='') as csvfile:
        for row in csv.reader(csvfile, delimiter=','):
            spring_variance_des.append(list(map(lambda x: float(x), row))[col])
            spring_variance_human.append(list(map(lambda x: float(x), row))[col+1])
            spring_occlusion += list(map(lambda x: float(x), row))[col+2]

    print(sen, mean(astar_variance_human), mean(spring_variance_human))
    des_variance_means.extend([mean(astar_variance_des), mean(spring_variance_des)])
    human_variance_means.extend([mean(astar_variance_human), mean(spring_variance_human)])
    occlusions.extend([float(astar_occlusion), float(spring_occlusion)])

combined_results = np.array([*des_variance_means, *human_variance_means, *occlusions])

current_results.append(combined_results)

result_means = np.mean(np.array(current_results[1:]).astype(float), axis=0)

xticks = []
plt.figure()
plt.title('Average Variance from Desired Location')
plt.rcParams["figure.figsize"] = [8.00, 4.50]
plt.rcParams["figure.autolayout"] = True
for i, scen in enumerate(scenarios):
    plt.bar(i*2, result_means[i*2], width=-0.8, label=scen, align='edge')
    plt.bar(i*2, result_means[i*2+1], width=0.8, label=scen, align='edge')
    xticks.extend([scen + ' AStar', scen + ' Spring'])
plt.ylabel("Mean")
plt.xlabel("Scenario")
plt.xticks(np.arange(-0.5, len(xticks), 1), xticks, rotation=45, ha="right")
plt.savefig('des_means.png')

xticks = []
plt.figure()
plt.title('Average Variance from Human Location')
plt.rcParams["figure.figsize"] = [8.00, 4.50]
plt.rcParams["figure.autolayout"] = True
for i, scen in enumerate(scenarios):
    plt.bar(i*2, result_means[i*2 + len(scenarios)], width=-0.8, label=scen, align='edge')
    plt.bar(i*2, result_means[i*2 + len(scenarios) + 1], width=0.8, label=scen, align='edge')
    xticks.extend([scen + ' AStar', scen + ' Spring'])
plt.axhline(y=2, color='r', linestyle='-')
plt.ylabel("Mean")
plt.xlabel("Scenario")
plt.xticks(np.arange(-0.5, len(xticks), 1), xticks, rotation=45, ha="right")
plt.savefig('human_means.png')

for i in range(len(combined_results)):
    if combined_results[i] != 0 and combined_results[i] == current_results[-2][i]:
        print(i, " Is the same as previous save")
        quit()

with open("results.csv", 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for result in current_results:
        writer.writerow(result)


