import math
import numpy as np

"Definition of the vectors and values for the algorithm"
ToF1_vector = [52.200001, 52.000000, 52.200001, 52.799999, 53.000000, 53.200001, 53.400002, 53.799999, 54.500000, 55.099998, 56.000000, 56.500000, 57.500000, 58.200001, 59.400002, 60.200001, 62.000000, 62.900002, 64.500000, 66.400002, 68.400002, 70.800003, 71.599998, 73.099998, 74.000000, 74.800003, 75.300003, 75.000000, 74.000000, 73.199997, 72.199997, 70.800003, 69.500000, 67.699997, 66.599998, 65.199997, 64.099998, 63.500000, 62.500000, 61.700001, 61.200001, 60.799999, 60.099998, 59.900002, 59.299999, 59.299999, 59.000000, 58.900002, 59.200001, 59.099998, 59.400002, 59.400002, 59.799999, 59.599998, 60.500000, 60.700001, 60.900002, 61.400002, 61.500000, 61.099998, 60.200001, 59.000000, 57.500000, 55.900002, 52.400002, 49.500000, 46.500000, 44.500000, 42.000000, 39.799999, 37.900002, 36.799999, 35.200001, 33.900002, 32.599998, 31.700001, 30.799999, 29.700001, 29.000000, 28.400000, 27.600000, 27.200001, 26.500000, 26.100000, 25.600000, 25.200001, 24.799999, 24.600000, 24.299999, 24.100000, 23.799999, 23.700001, 23.400000, 23.299999, 23.100000, 23.000000, 22.900000, 23.000000, 22.799999, 22.900000, 22.900000, 22.900000, 22.900000, 23.000000, 23.200001, 23.299999, 23.500000, 23.600000, 23.799999, 24.100000, 24.400000, 24.700001, 25.200001, 25.600000, 26.100000, 26.400000, 26.600000, 26.799999, 26.500000, 26.299999, 26.100000, 25.400000, 24.700001, 24.000000, 22.799999, 21.799999, 21.100000, 20.500000, 19.900000, 19.200001, 18.600000, 18.500000, 17.799999, 17.299999, 17.100000, 16.900000, 16.500000, 16.400000, 16.100000, 15.900000, 15.700000, 15.500000, 15.500000, 15.300000, 15.400000, 15.100000, 15.100000, 15.200000, 15.200000, 15.100000, 15.000000, 15.100000, 15.100000, 15.300000, 15.200000, 15.300000, 15.300000, 15.600000, 15.700000, 15.900000, 16.100000, 16.299999, 16.500000, 16.900000, 17.100000, 17.500000, 17.900000, 18.299999, 18.600000, 19.100000, 19.500000, 20.000000, 20.600000, 21.200001, 21.799999, 22.600000, 23.299999, 24.000000, 25.000000, 26.000000, 27.299999, 28.700001, 30.500000, 32.299999, 35.299999, 37.900002, 41.700001, 43.400002, 47.299999, 49.299999, 51.099998, 51.799999, 52.400002, 52.700001, 52.299999, 52.500000, 52.200001, 52.000000, 52.000000, 52.200001]
ToF2_vector = [23.799999, 23.600000, 23.299999, 22.900000, 22.400000, 22.100000, 21.700001, 21.299999, 21.200001, 20.799999, 20.600000, 20.400000, 20.100000, 20.000000, 20.100000, 20.000000, 19.799999, 19.900000, 19.700001, 19.700001, 19.799999, 19.799999, 19.900000, 19.900000, 20.100000, 20.200001, 20.299999, 20.400000, 20.600000, 21.000000, 21.299999, 21.400000, 21.799999, 22.200001, 22.600000, 23.200001, 23.500000, 24.100000, 24.700001, 25.299999, 26.000000, 26.799999, 27.600000, 28.500000, 29.600000, 30.500000, 31.799999, 32.900002, 34.400002, 36.099998, 38.200001, 40.500000, 43.599998, 47.400002, 51.299999, 54.299999, 57.000000, 59.000000, 60.299999, 61.099998, 61.900002, 62.299999, 62.000000, 61.799999, 61.599998, 61.500000, 61.500000, 61.500000, 61.700001, 61.400002, 61.599998, 62.000000, 62.099998, 62.200001, 62.500000, 63.200001, 63.900002, 64.800003, 65.599998, 66.000000, 67.400002, 68.000000, 69.500000, 70.500000, 72.000000, 73.699997, 75.199997, 76.300003, 77.400002, 78.099998, 78.400002, 77.900002, 75.500000, 72.900002, 69.800003, 67.199997, 63.299999, 61.900002, 60.099998, 60.000000, 59.200001, 58.799999, 58.599998, 58.700001, 58.599998, 58.599998, 58.400002, 58.099998, 57.099998, 56.599998, 56.299999, 55.599998, 55.400002, 55.400002, 54.700001, 54.900002, 54.700001, 54.900002, 54.799999, 54.900002, 54.900002, 54.799999, 54.599998, 54.299999, 53.299999, 51.700001, 49.400002, 47.299999, 43.900002, 40.000000, 36.500000, 34.299999, 31.400000, 29.299999, 27.700001, 26.200001, 24.900000, 23.900000, 22.900000, 22.000000, 21.299999, 20.500000, 19.799999, 19.100000, 18.400000, 17.700001, 17.200001, 17.000000, 16.500000, 16.100000, 15.700000, 15.400000, 15.300000, 14.900000, 14.600000, 14.600000, 14.300000, 14.100000, 13.900000, 13.900000, 13.700000, 13.600000, 13.400000, 13.400000, 13.300000, 13.200000, 13.300000, 13.100000, 13.100000, 13.100000, 13.300000, 13.300000, 13.300000, 13.400000, 13.500000, 13.500000, 13.800000, 13.800000, 14.000000, 14.100000, 14.400000, 14.700000, 14.900000, 15.200000, 15.400000, 15.800000, 16.200001, 16.500000, 17.100000, 17.600000, 18.299999, 18.799999, 20.000000, 20.600000, 21.700001, 22.400000, 23.299999, 23.600000, 23.799999, 23.900000]

#ToF1_vector = [44.099998, 44.200001, 44.400002, 44.299999, 44.700001, 44.900002, 45.400002, 45.500000, 46.099998, 46.799999, 47.299999, 47.900002, 48.599998, 49.299999, 49.900002, 51.299999, 52.099998, 53.400002, 54.900002, 55.599998, 57.500000, 58.900002, 60.599998, 62.599998, 64.599998, 67.000000, 69.800003, 72.800003, 76.599998, 79.699997, 84.099998, 86.800003, 88.800003, 91.599998, 92.800003, 92.900002, 92.300003, 90.900002, 90.300003, 88.099998, 85.199997, 82.800003, 80.400002, 79.500000, 77.800003, 76.400002, 74.800003, 73.300003, 72.199997, 71.300003, 70.300003, 70.099998, 69.000000, 68.400002, 67.500000, 67.000000, 67.099998, 66.900002, 65.699997, 64.800003, 63.900002, 62.299999, 60.500000, 58.400002, 55.599998, 52.900002, 50.000000, 47.900002, 45.299999, 43.700001, 41.799999, 40.299999, 38.799999, 37.299999, 36.299999, 35.299999, 34.000000, 33.200001, 32.500000, 31.799999, 31.100000, 30.500000, 29.900000, 29.400000, 28.900000, 28.600000, 28.200001, 27.799999, 27.400000, 27.100000, 26.600000, 26.400000, 26.299999, 26.100000, 26.000000, 25.900000, 26.000000, 25.900000, 25.900000, 25.900000, 25.900000, 26.100000, 26.000000, 26.299999, 26.400000, 26.500000, 26.799999, 26.900000, 27.299999, 27.500000, 27.799999, 28.000000, 28.299999, 28.400000, 28.700001, 29.000000, 29.100000, 29.100000, 28.799999, 27.799999, 26.299999, 24.500000, 23.400000, 22.500000, 21.700001, 20.900000, 20.600000, 20.000000, 19.400000, 18.900000, 18.500000, 18.100000, 17.799999, 17.400000, 17.200001, 16.900000, 16.600000, 16.400000, 16.200001, 16.000000, 15.700000, 15.700000, 15.400000, 15.300000, 15.100000, 15.000000, 14.700000, 14.700000, 14.700000, 14.700000, 14.800000, 15.000000, 15.200000, 15.300000, 15.300000, 15.600000, 15.600000, 15.800000, 16.100000, 15.900000, 16.400000, 16.600000, 16.900000, 17.100000, 17.400000, 17.700001, 18.000000, 18.299999, 18.799999, 19.299999, 19.900000, 20.100000, 20.900000, 21.700001, 22.299999, 23.299999, 24.000000, 25.100000, 25.700001, 25.800001, 26.900000, 27.500000, 27.799999, 29.000000, 30.200001, 33.299999, 37.000000, 40.099998, 43.200001, 44.000000, 44.900002, 44.900002, 45.200001, 45.000000, 45.000000, 45.200001, 44.799999, 44.799999, 44.299999, 44.299999]
#ToF2_vector = [29.200001, 28.400000, 27.700001, 27.100000, 26.700001, 26.200001, 25.700001, 25.400000, 25.100000, 24.799999, 24.500000, 24.299999, 24.100000, 24.000000, 23.700001, 23.700001, 23.500000, 23.600000, 23.400000, 23.400000, 23.500000, 23.600000, 23.600000, 23.700001, 23.900000, 24.000000, 24.100000, 24.400000, 24.799999, 25.000000, 25.200001, 25.500000, 26.000000, 26.400000, 26.900000, 27.200001, 27.900000, 28.400000, 29.000000, 29.700001, 30.600000, 31.400000, 32.200001, 33.099998, 34.299999, 35.500000, 37.000000, 39.000000, 41.099998, 43.799999, 45.900002, 48.700001, 52.200001, 55.299999, 58.099998, 60.200001, 62.000000, 62.900002, 63.299999, 63.500000, 62.900002, 61.799999, 60.599998, 59.099998, 57.799999, 56.599998, 55.599998, 55.000000, 53.700001, 53.000000, 52.099998, 51.500000, 50.599998, 50.200001, 49.700001, 49.500000, 49.099998, 48.900002, 48.799999, 48.200001, 48.200001, 48.200001, 48.000000, 48.000000, 48.000000, 47.799999, 47.700001, 47.500000, 46.900002, 46.400002, 46.500000, 47.200001, 48.000000, 48.200001, 49.799999, 50.599998, 51.900002, 52.700001, 53.099998, 53.299999, 52.900002, 52.299999, 51.700001, 51.099998, 50.200001, 49.299999, 48.900002, 48.599998, 47.799999, 47.200001, 46.900002, 46.599998, 46.200001, 45.700001, 45.799999, 45.700001, 45.500000, 45.400002, 45.700001, 45.599998, 45.599998, 45.799999, 45.799999, 46.200001, 46.900002, 47.299999, 47.599998, 48.299999, 48.400002, 49.500000, 50.299999, 51.099998, 52.000000, 53.400002, 54.799999, 55.200001, 56.799999, 58.599998, 59.200001, 59.299999, 59.599998, 60.000000, 58.900002, 57.299999, 55.799999, 52.900002, 51.799999, 52.700001, 50.000000, 49.700001, 48.500000, 47.400002, 46.700001, 45.799999, 45.099998, 45.000000, 44.400002, 44.000000, 43.799999, 43.299999, 43.000000, 43.400002, 43.000000, 42.799999, 42.799999, 43.000000, 42.799999, 43.200001, 43.700001, 43.500000, 44.200001, 43.900002, 44.799999, 45.000000, 45.900002, 45.700001, 46.400002, 47.799999, 48.599998, 49.099998, 49.200001, 48.200001, 47.099998, 46.000000, 44.700001, 43.400002, 41.799999, 41.299999, 39.400002, 38.700001, 37.200001, 36.599998, 36.700001, 35.799999, 35.000000, 34.000000, 32.700001, 31.900000, 30.900000, 30.000000]

theorical_distance = 54 #centimeters
theorical_angle = 32.96 #This is a value that will be needed to decide the final rotation angle

num_samples = 200
added_angle = 360 / num_samples

"""Start of the algorithm: """

"""Mass center substraction to center both figures, so they can be rotated one over the other"""
def ToF2Matrix(ToF):
    angles = [math.radians(i * added_angle) for i in range(len(ToF))]
    initial_coordinates = [[ToF[i] * math.cos(angles[i]), 
                          ToF[i] * math.sin(angles[i])] for i in range(0, len(ToF))]
    return initial_coordinates

def mass_center_substraction(ToF1, ToF2):
    initial_coordinates1 = ToF2Matrix(ToF1_vector)
    initial_coordinates2 = ToF2Matrix(ToF2_vector)

    mass_center1 = [0, 0]
    mass_center2 = [0, 0]
    weights = [0, 0, 0, 0]

    for i in range(0, len(ToF1)):
        mass_center1[0] += initial_coordinates1[i][0] * (abs(initial_coordinates1[i-1][0] - initial_coordinates1[(i+1)%len(ToF1)][0]))
        weights[0] += (abs(initial_coordinates1[i-1][0] - initial_coordinates1[(i+1)%len(ToF1)][0]))
        mass_center1[1] += initial_coordinates1[i][1] * (abs(initial_coordinates1[i-1][1] - initial_coordinates1[(i+1)%len(ToF1)][1]))
        weights[1] += (abs(initial_coordinates1[i-1][1] - initial_coordinates1[(i+1)%len(ToF1)][1]))
        mass_center2[0] += initial_coordinates2[i][0] * (abs(initial_coordinates2[i-1][0] - initial_coordinates2[(i+1)%len(ToF2)][0]))
        weights[2] += (abs(initial_coordinates2[i-1][0] - initial_coordinates2[(i+1)%len(ToF1)][0]))
        mass_center2[1] += initial_coordinates2[i][1] * (abs(initial_coordinates2[i-1][1] - initial_coordinates2[(i+1)%len(ToF2)][1]))
        weights[3] += (abs(initial_coordinates2[i-1][1] - initial_coordinates2[(i+1)%len(ToF1)][1]))

    mass_center1[0] /= weights[0]
    mass_center1[1] /= weights[1]
    mass_center2[0] /= weights[2]
    mass_center2[1] /= weights[3]

    figure1_centered = []
    figure2_centered = []

    for j in range(0, len(ToF1)):
        figure1_centered.append([initial_coordinates1[j][0] - mass_center1[0], 
                                initial_coordinates1[j][1] - mass_center1[1]])
        figure2_centered.append([initial_coordinates2[j][0] - mass_center2[0], 
                                initial_coordinates2[j][1] - mass_center2[1]])
    return figure1_centered, figure2_centered, mass_center1, mass_center2 

figure1_matrix, figure2_matrix, mass_center1, mass_center2 = mass_center_substraction(ToF1_vector, ToF2_vector)


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

"""Interpolate both matrix and fulfill them with points"""
figure1_interpolated = interpolate_points(figure1_matrix)
figure2_interpolated = interpolate_points(figure2_matrix)

"""The idea is to calculate the minimum distance from every point of figure1 to every point in figure2 and calculate the minimum. """
def correlation(figure1, figure2):
    correlation_value = 0
    for (x1, y1) in figure1:
        min_distance = float('inf')
        for (x2, y2) in figure2:
            distance_square = ((x2-x1)**2 + (y2-y1)**2) # To optimize the algorithm, the square root is not done --> minimum d**0.5 = minimum d
            if distance_square < min_distance:
                min_distance = distance_square
        correlation_value += min_distance
    return correlation_value

def rotate_matrix(matrix, theta):
    cos_theta, sin_theta = np.cos(theta), np.sin(theta)
    rotated_matrix = []
    for (x, y) in matrix:
        x_rot = x * cos_theta - y * sin_theta
        y_rot = x * sin_theta + y * cos_theta
        rotated_matrix.append((x_rot, y_rot))
    return rotated_matrix

def rotate_single_point(x, y, theta):
    cos_theta, sin_theta = math.cos(theta), math.sin(theta)
    x_rot = x * cos_theta - y * sin_theta
    y_rot = x * sin_theta + y * cos_theta
    return (x_rot, y_rot)

def reduce_points(points, step=3): #   (number of points) / step = new number of points
    #reduced_points = points[::step]
    # Ensure the figure is completed 
    #if reduced_points[-1] != points[-1]:
    #    reduced_points.append(points[-1])
    return points[::step]


def calculate_orientation_angle(figure1, figure2):
    long = len(figure2) #Because is the figure we are going to rotate
    angle = 360 / long
    rotation_angle = []
    #min_correlation_value = float('inf')
    min_correlation_value = []
    for i in range(0, len(figure2)):
        actual_correlation_value = correlation(figure1, figure2)
        min_correlation_value.append((actual_correlation_value, angle*i))
        figure2 = rotate_matrix(figure2, -(angle*(2*np.pi)/360))
    for j in range(0, len(figure2)):
        if (min_correlation_value[j][0] < min_correlation_value[(j+1)%len(figure2)][0] and min_correlation_value[j][0] < min_correlation_value[(j-1)%len(figure2)][0]):
            rotation_angle.append(min_correlation_value[j][1])
    return rotation_angle

figure1_interpolated_reduced = reduce_points(figure1_interpolated)
figure2_interpolated_reduced = reduce_points(figure2_interpolated)

final_rotation_angle = calculate_orientation_angle(figure1_interpolated_reduced, figure2_interpolated_reduced)
min_rot_angle = float('inf')
final_angle = 0

for i in range(0, len(final_rotation_angle)):
    if (final_rotation_angle[i] - theorical_angle) < min_rot_angle:
        final_angle = final_rotation_angle[i]
        min_rot_angle = final_rotation_angle[i] - theorical_angle
figure2_final = rotate_matrix(figure2_interpolated_reduced, -final_angle*(2*np.pi)/360)

print(f"The theorical displacement was {theorical_angle} and the final rotation angle is {final_angle}")

mass_center2_rotated = rotate_single_point(mass_center2[0], mass_center2[1], (90-final_angle))
practical_distance = ((mass_center1[0] - mass_center2_rotated[0])**2 + (mass_center1[1] - mass_center2_rotated[1])**2)**0.5

print(f"The theorical distance is {theorical_distance}, the real distance calculated is {practical_distance}")
