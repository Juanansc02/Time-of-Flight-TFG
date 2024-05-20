import matplotlib.pyplot as plt
import numpy as np
import math

num_samples = 200
sensor_distance_limit = 2
robot_position1 = [0, 0]
robot_position2 = [0, 0]
vertices = [0, 0]
added_angle = 0

"""This script has all the functions and constants used in the algorithms for square, closed convex and open convex"""

def reduce_points(points, step=2): #   (number of points) / step = new number of points
    reduced_points = points[::step]
    # Ensure the figure is completed 
    if reduced_points[-1] != points[-1]:
        reduced_points.append(points[-1])
    return reduced_points

def rotate_matrix(matrix, theta):
    theta = np.radians(theta)
    cos_theta, sin_theta = np.cos(theta), np.sin(theta)
    rotated_matrix = []
    for (x, y) in matrix:
        x_rot = x * cos_theta - y * sin_theta
        y_rot = x * sin_theta + y * cos_theta
        rotated_matrix.append((x_rot, y_rot))
    return rotated_matrix

def distance_btw_points(point1, point2):
    return ((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)**0.5

def reconstruct_figure_from_matrix(points1, points2):
    points1.append(points1[0])
    points2.append(points2[0])
    points1 = np.array(points1)
    points2 = np.array(points2)

    plt.figure()
    plt.plot(0, 0, 'bo', markersize = 1)
    plt.plot(points1[:, 0], points1[:, 1], 'go', markersize = 1)  
    plt.plot(points2[:, 0], points2[:, 1], 'bo', markersize = 1)  
    plt.legend()
    plt.axis('equal')
    plt.show()

def Tof2Points(ToF1, ToF2):
    angles = np.linspace(0, 2 * np.pi, num_samples, endpoint=False)
    
    for i in range(0, len(ToF1)):
        if ToF1[i] > sensor_distance_limit:
            ToF1[i] = 0
    for j in range(0, len(ToF2)):
        if ToF2[j] > sensor_distance_limit:
            ToF2[j] = 0  
    
    points_x1 = [distance * np.cos(angle) for distance, angle in zip(ToF1, angles)]
    points_y1 = [distance * np.sin(angle) for distance, angle in zip(ToF1, angles)]
    points_x2 = [distance * np.cos(angle) for distance, angle in zip(ToF2, angles)]
    points_y2 = [distance * np.sin(angle) for distance, angle in zip(ToF2, angles)]

    points1 = []
    points2 = []

    for x, y in zip(points_x1, points_y1):
        points1.append((x, y))
    for x, y in zip(points_x2, points_y2):
        points2.append((x, y))
    
    points1 = [(x, y) for x, y in points1 if not (x == 0 and y == 0)]
    points2 = [(x, y) for x, y in points2 if not (x == 0 and y == 0)]

    return points1, points2

def intersection_point_calc(robot_position, unitary_vector, vertex1, vertex2):
    #Modify input data to numpy array for future calculations
    robot_position = np.array(robot_position)
    unitary_vector = np.array(unitary_vector)
    vertex1 = np.array(vertex1)
    vertex2 = np.array(vertex2)
    
    v1 = robot_position - vertex1
    v2 = vertex2 - vertex1
    v3 = np.array([-unitary_vector[1], unitary_vector[0]])
    
    t1 = np.cross(v2, v1) / np.dot(v2, v3) #Distance from point to segment. This value shall be finite and positive
    t2 = np.dot(v1, v3) / np.dot(v2, v3) #Ensures the intersection ocurrs in the segment we are looking for
    #print(np.dot(v1, v3))

    if t1 >= 0 and 0 <= t2 <= 1:
        return robot_position + t1 * unitary_vector
    else:
        return None

def calculate_tof(robot_position, vertices):
    distances = []
    angles = np.linspace(0, 2 * np.pi, num_samples, endpoint=False) #list of angles (in radians) from 0 to 2*pi
    
    for angle in angles:
        unitary_vector = np.array([np.cos(angle), np.sin(angle)]) #matrix with cosinus and sinus for each angle defined before
        closest_distance = None
        
        for i in range(len(vertices)):
            vertex1 = vertices[i]
            vertex2 = vertices[(i + 1) % len(vertices)]
            intersection_point = intersection_point_calc(robot_position, unitary_vector, vertex1, vertex2)
            if intersection_point is not None:
                distance = np.linalg.norm(intersection_point - np.array(robot_position))
                #distance = np.linalg.norm(intersection_point - np.array(robot_position)) #substraction of final_position - initial_position to obtain the distance to the environment
                if closest_distance is None or distance < closest_distance:
                    closest_distance = distance
        distances.append(closest_distance if closest_distance is not None else np.inf)
    
    return distances

def ToF2Matrix(ToF):
    angles = [math.radians(i * added_angle) for i in range(len(ToF))]
    initial_coordinates = [[ToF[i] * math.cos(angles[i]), 
                          ToF[i] * math.sin(angles[i])] for i in range(0, len(ToF))]
    return initial_coordinates

def interpolate_points(points):
    # Determining minimum distance between points
    distances = [np.linalg.norm(np.array(points[i]) - np.array(points[i+1])) for i in range(len(points)-1)]
    minimum_distance = min(distances)
    
    interpolated_points = [points[0]]  
    for i in range(len(points) - 1):
        point1 = points[i]
        point2 = points[i + 1]
        # Distance between points
        actual_distance = np.linalg.norm(np.array(point2) - np.array(point1))
        
        # Number of interpolations between points
        num_interpolated_points = int(actual_distance / minimum_distance) - 1
        
        for j in range(1, num_interpolated_points + 1):
            t = j / (num_interpolated_points + 1)
            x_interpolated = point1[0] + t * (point2[0] - point1[0])
            y_interpolated = point1[1] + t * (point2[1] - point1[1])
            interpolated_points.append((x_interpolated, y_interpolated))
        
        # Add real points to maintain the data from the initial ToF vector
        interpolated_points.append(point2)
    
    return interpolated_points