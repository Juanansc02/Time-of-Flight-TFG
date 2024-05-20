import matplotlib.pyplot as plt
import numpy as np
import math
from utils.utils import reduce_points, rotate_matrix, distance_btw_points, reconstruct_figure_from_matrix, Tof2Points, calculate_tof, interpolate_points
num_samples = 200
added_angle = 360 / num_samples
sensor_distance_limit = 2

robot_position1 = [0, 0]
robot_position2 = [1, 4]

initial_seed = [1.5, 3, -12]
robot_position1[0] += initial_seed[0]
robot_position1[1] += initial_seed[1]

#Descent incrementals
dx = 0.05
dy= 0.05
da = 0.2

vertices = [(3, 0), (0, 3), (0, 6), (3, 9), (6, 9), (9, 6), (9, 3), (6, 0)]

"""Functions to verify functionality of the algorithm"""

ToF1 = calculate_tof(robot_position1, vertices)
ToF2 = calculate_tof(robot_position2, vertices)

points1, points2 = Tof2Points(ToF1, ToF2)

figure1 = interpolate_points(points1)
figure2 = rotate_matrix(interpolate_points(points2), initial_seed[2])

#Do this to avoid problem with excess of points when interpolating 
r1 = int(len(figure1) / len(figure2))
r2 = int(len(figure2) / len(figure1))

if (r1 > 1.5):
    figure1 = reduce_points(figure1, r1)
else:
    figure2 = reduce_points(figure2, r2)

"""
Secondly, Housdorff distance has been defined to obtain the distance from ToF1 to ToF2
"""
def displaceX(points, dx):
    new_points = []
    for x, y in points:
        new_points.append([x + dx, y]) 
    return new_points

def displaceY(points, dy):
    new_points = []
    for x, y in points:
        new_points.append([x, y + dy])  
    return new_points

def hausdorff_distance(points1, points2): 
    distance = 0
    distance1 = 0
    distance2 = 0
    d1_min = float('inf')
    d2_min = float('inf')
    for i in points1:
        d1_min = float('inf')
        for j in points2:
            distance = distance_btw_points(i, j)
            if (d1_min > distance):
                d1_min = distance
        distance1 += d1_min
    
    for i in points2:
        d2_min = float('inf')
        for j in points1:
            distance = distance_btw_points(i, j)
            if (d2_min > distance):
                d2_min = distance
        distance2 += d2_min
    
    return ((distance1 + distance2) / (len(points1) + len(points2)))



def displace_points(points, dx, dy):
    return [(x + dx, y + dy) for x, y in points]

def near_points(matrix1, matrix2, seed):
    common_points1 = []
    common_points2 = []

    for point2 in matrix2:
        if distance_btw_points([0, 0], point2) < ratio: common_points2.append(point2)
    for point1 in matrix1:
        if distance_btw_points(seed, point1) < ratio: common_points1.append(point1)

    return common_points1, common_points2

seed = [0, 0]  
ratio = 2  

figure1_p, figure2_p = near_points(figure1, figure2, seed) #Ultima modificacion


""" Now, the main starts: """

end = 0
i = 0

x_disp = 0
y_disp = 0
a_disp = 0

while not end:
    
    d = hausdorff_distance(figure1_p, figure2_p)
    d_plusX = hausdorff_distance(displaceX(figure1_p, dx), figure2_p)
    d_minusX = hausdorff_distance(displaceX(figure1_p, -dx), figure2_p)
    d_plusY = hausdorff_distance(displaceY(figure1_p, dy), figure2_p)
    d_minusY = hausdorff_distance(displaceY(figure1_p, -dy), figure2_p)
    d_plusA = hausdorff_distance(rotate_matrix(figure1_p, da), figure2_p)
    d_minusA = hausdorff_distance(rotate_matrix(figure1_p, -da), figure2_p)
    
    if (d_minusX > d and d_plusX > d  and d_minusY > d and d_plusY > d and d_minusA > d and d_plusA > d or i > 150):
        end = 1
        break

    if (d > d_plusX):
        figure1_p = displaceX(figure1_p, dx)
        figure1 = displaceX(figure1, dx)
        seed[0] -= dx
        x_disp -= dx
    elif (d > d_minusX):
        figure1_p = displaceX(figure1_p, -dx)
        figure1 = displaceX(figure1, -dx)
        seed[0] += dx
        x_disp += dx
    if (d > d_plusY):
        figure1_p = displaceY(figure1_p, dy)
        figure1 = displaceY(figure1, dy)
        seed[1] -= dy
        y_disp -= dy
    elif (d > d_minusY):
        figure1_p = displaceY(figure1_p, -dy)
        figure1 = displaceY(figure1, -dy)
        seed[1] += dy
        y_disp += dy
    if (d > d_plusA):
        figure1_p = rotate_matrix(figure1_p, da)
        figure1 = rotate_matrix(figure1, da)
        a_disp -= da
    elif (d > d_minusA):
        figure1_p = rotate_matrix(figure1_p, -da)
        figure1 = rotate_matrix(figure1, -da)
        a_disp += da
    
    if i % 20 == 0:
        print(len(figure1_p), len(figure2_p))
        reconstruct_figure_from_matrix(figure1_p, figure2_p)
    i += 1
    print(f"Iteration {i}: [{x_disp}, {y_disp}, {a_disp}]")
    seed[0] = -seed[0]
    seed[1] = -seed[1]
    figure1_p, figure2_p = near_points(figure1, figure2, seed)

print("Gradient Descent finished")
reconstruct_figure_from_matrix(figure1, figure2)