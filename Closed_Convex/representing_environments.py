# Re-importing necessary libraries after reset
import matplotlib.pyplot as plt
import numpy as np

num_samples = 72

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
                distance = np.linalg.norm(intersection_point - np.array(robot_position)) #substraction of final_position - initial_position to obtain the distance to the environment
                if closest_distance is None or distance < closest_distance:
                    closest_distance = distance
        distances.append(closest_distance if closest_distance is not None else np.inf)
    
    return distances

robot_position = (0.5, 2.2)
#vertices = [(0, 0), (0, 5), (8, 5), (8, 0)]
#vertices = [(0, 0), (2.5, 5), (5, 0)]
vertices = [(2, 0), (0, 2), (0, 4), (2, 6), (4, 6), (6, 4), (6, 2), (4, 0)]

tof_distances = calculate_tof(robot_position, vertices)

def reconstruct_figure_from_tof(robot_position, distances):
    angles = np.linspace(0, 2 * np.pi, num_samples, endpoint=False)
    points_x = [robot_position[0] + distance * np.cos(angle) for distance, angle in zip(distances, angles)]
    points_y = [robot_position[1] + distance * np.sin(angle) for distance, angle in zip(distances, angles)]

    vertices_array = np.array(vertices + [vertices[0]])
    #plt.plot(vertices_array[:, 0], vertices_array[:, 1], '-o', label='Original Geometric Figure')
    plt.plot(points_x + [points_x[0]], points_y + [points_y[0]], 'bo', markersize = 1, label='Reconstructed Figure')
    plt.plot(robot_position[0], robot_position[1], 'ro', label='Robot Position')

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.legend()
    plt.axis('equal')
    plt.title('Original and Reconstructed Geometric Figure')
    plt.show()

reconstruct_figure_from_tof(robot_position, tof_distances)