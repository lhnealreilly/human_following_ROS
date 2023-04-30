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
        current_results.append([float(x) for x in row] if i > 1 else [x for x in row])

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
    occlusions.extend([astar_occlusion, spring_occlusion])

combined_results = [*des_variance_means, *human_variance_means, *occlusions]
for i in range(len(combined_results)):
    if combined_results[i] == current_results[-1][i]:
        print(i, " Is the same as previous save")
        quit()
current_results.append(combined_results)

with open("results.csv", 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for result in current_results:
        writer.writerow(result)


