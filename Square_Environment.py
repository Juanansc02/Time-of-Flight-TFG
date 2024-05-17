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

visualize_environment()