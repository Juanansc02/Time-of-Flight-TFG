import matplotlib.pyplot as plt
import numpy as np
import math
from random import randint

square_height = 5
square_lenght = 8
samples = 36
added_angle = 360 / samples

robot_position1 = [6, 4]
robot_position2 = [1, 1]

def visualize_environment():
    x = [0, 0, 8, 8, 0]
    y = [0, 5, 5, 0, 0]

    plt.plot(x, y, 'b-')
    plt.scatter(robot_position1[0], robot_position1[1], c='red', marker='o')
    plt.scatter(robot_position2[0], robot_position2[1], c='green', marker='o')

    plt.title('Environment')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()

#visualize_environment()

def determine_distance(robot_position, square_height, square_lenght):
    
    distance = []
    alpha = 0
    radians = math.radians(alpha)

    #First Quadrant
    while (square_lenght > robot_position[0] + (square_height-robot_position[1])*math.tan(radians)):
        H = (square_height - robot_position[1]) / math.cos(radians)
        distance.append(H)
        alpha = (alpha + added_angle) % 360
        radians = math.radians(alpha)
    
    beta = 90 - alpha
    radians = math.radians(beta)

    while (beta > 0):
        H = (square_lenght - robot_position[0]) / math.cos(radians)
        distance.append(H)
        alpha = (alpha + added_angle) % 360
        beta = 90 - alpha
        radians = math.radians(beta)
    
    #Fourth Quadrant
    alpha = 0
    radians = math.radians(alpha)
    
    while (robot_position[1] - (square_lenght-robot_position[0])*math.tan(radians) > 0):
        H = (square_lenght-robot_position[0]) / math.cos(radians)
        distance.append(H)
        alpha = (alpha + added_angle) % 360
        radians = math.radians(alpha)
    
    beta = 90 - alpha
    radians = math.radians(beta)

    while (beta > 0):
        H = (robot_position[1]) / math.cos(radians)
        distance.append(H)
        alpha = (alpha + added_angle) % 360
        beta = 90 - alpha
        radians = math.radians(beta)

    #Third Quadrant
    alpha = 0
    radians = math.radians(alpha)

    while (robot_position[0] - (robot_position[1])*math.tan(radians) > 0):
        H = (robot_position[1]) / math.cos(radians)
        distance.append(H)
        alpha = (alpha + added_angle) % 360
        radians = math.radians(alpha)
    
    beta = 90 - alpha
    radians = math.radians(beta)

    while (beta > 0):
        H = (robot_position[0]) / math.cos(radians)
        distance.append(H)
        alpha = (alpha + added_angle) % 360
        beta = 90 - alpha
        radians = math.radians(beta)

    #Second Quadrant
    alpha = 0
    radians = math.radians(alpha)
    
    while (square_height > robot_position[1] + (robot_position[0])*math.tan(radians)):
        H = (robot_position[0]) / math.cos(radians)
        distance.append(H)
        alpha = (alpha + added_angle) % 360
        radians = math.radians(alpha)
    
    beta = 90 - alpha
    radians = math.radians(beta)

    while (beta > 0):
        H = (square_height - robot_position[1]) / math.cos(radians)
        distance.append(H)
        alpha = (alpha + added_angle) % 360
        beta = 90 - alpha
        radians = math.radians(beta)
    return distance

#Calculate Time of Fly for position 1 and 2:
ToF1 = determine_distance(robot_position1, square_height, square_lenght)
ToF2 = determine_distance(robot_position2, square_height, square_lenght)
rounded_ToF1 = [round(valor1, 2) for valor1 in ToF1]
rounded_ToF2 = [round(valor2, 2) for valor2 in ToF2]

#Second position of the robot is re-oriented to consider orientation
#displacement = randint(0, samples)
displacement = 30
ToF2_displaced = ToF2[displacement:] + ToF2[:displacement]
rounded_ToF2_displaced = [round(valor, 2) for valor in ToF2_displaced]

#Starting the algorithm:

#Distance traveled by the robot
distance_traveled = math.sqrt(math.pow(abs(robot_position2[0]-robot_position1[0]), 2) + math.pow(abs(robot_position2[1]-robot_position1[1]), 2))
#print(f"The distance traveled by the robot is {round(distance_traveled, 2)}m")

#Graphic representation of ToF points in graphic:
def represent_figure(ToF1, ToF2):
    angles = [math.radians(i * added_angle) for i in range(len(ToF1))]
    
    #Define the points that need to be graphicated:
    coordinates1 = [[ToF1[i] * math.sin(angles[i]), 
                          ToF1[i] * math.cos(angles[i])] for i in range(0, len(ToF1))]
    coordinates2 = [[ToF2[j]*math.sin(angles[j]), 
                          ToF2[j]*math.cos(angles[j])] for j in range(0, len(ToF2))]
    
    plt.plot(0, 0, 'bo')

    plt.plot([coordinates1[i][0] for i in range(0, len(ToF1))], 
             [coordinates1[i][1] for i in range(0, len(ToF1))], 'ro')
    plt.plot([coordinates2[i][0] for i in range(0, len(ToF2))], 
             [coordinates2[i][1] for i in range(0, len(ToF2))], 'go')
    #plt.plot((0, 0, 8, 8, 0), (0, 5, 5, 0, 0), 'b-')

    plt.title('Detected environment')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.gca().set_aspect('equal')
    plt.show()
    

#represent_figure(rounded_ToF1, rounded_ToF2_displaced)

#Once the figure is represented, the MASS CENTER shall be calculated to obtain a reference
def mass_center_substraction(ToF1, ToF2):
    angles = [math.radians(i * added_angle) for i in range(len(ToF1))]
    
    #Initial square points:
    initial_coordinates1 = [[ToF1[i] * math.sin(angles[i]), 
                          ToF1[i] * math.cos(angles[i])] for i in range(0, len(ToF1))]
    initial_coordinates2 = [[ToF2[j]*math.sin(angles[j]), 
                          ToF2[j]*math.cos(angles[j])] for j in range(0, len(ToF2))]
    
    mass_center1 = [0, 0]
    mass_center2 = [0, 0]
    weights = [0, 0, 0, 0]

    for i in range(0, len(ToF1)):
        mass_center1[0] += initial_coordinates1[i][0] * (abs(initial_coordinates1[i][0] - initial_coordinates1[(i+1)%len(ToF1)][0]))
        weights[0] += (abs(initial_coordinates1[i][0] - initial_coordinates1[(i+1)%len(ToF1)][0]))
        mass_center1[1] += initial_coordinates1[i][1] * (abs(initial_coordinates1[i][1] - initial_coordinates1[(i+1)%len(ToF1)][1]))
        weights[1] += (abs(initial_coordinates1[i][1] - initial_coordinates1[(i+1)%len(ToF1)][1]))
        mass_center2[0] += initial_coordinates2[i][0] * (abs(initial_coordinates2[i][0] - initial_coordinates2[(i+1)%len(ToF2)][0]))
        weights[2] += (abs(initial_coordinates2[i][0] - initial_coordinates2[(i+1)%len(ToF1)][0]))
        mass_center2[1] += initial_coordinates2[i][1] * (abs(initial_coordinates2[i][1] - initial_coordinates2[(i+1)%len(ToF2)][1]))
        weights[3] += (abs(initial_coordinates2[i][1] - initial_coordinates2[(i+1)%len(ToF1)][1]))

    mass_center1[0] /= weights[0]
    mass_center1[1] /= weights[1]
    mass_center2[0] /= weights[2]
    mass_center2[1] /= weights[3]

    square_centered1 = []
    square_centered2 = []

    for j in range(0, len(ToF1)):
        square_centered1.append([initial_coordinates1[j][0] - mass_center1[0], 
                                initial_coordinates1[j][1] - mass_center1[1]])
        square_centered2.append([initial_coordinates2[j][0] - mass_center2[0], 
                                initial_coordinates2[j][1] - mass_center2[1]])

    plt.plot(0, 0, 'bo')

    plt.plot([square_centered1[i][0] for i in range(0, len(ToF1))], 
             [square_centered1[i][1] for i in range(0, len(ToF1))], 'r-')
    plt.plot([square_centered2[i][0] for i in range(0, len(ToF2))], 
             [square_centered2[i][1] for i in range(0, len(ToF2))], 'g-')
    #plt.plot((0, 0, 8, 8, 0), (0, 5, 5, 0, 0), 'b-')
    print(initial_coordinates1)
    plt.title('Detected environment')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.gca().set_aspect('equal')
    plt.grid(True)
    plt.show()
    return square_centered1, square_centered2    

sqr1, squr2 = mass_center_substraction(rounded_ToF1, rounded_ToF2_displaced)
