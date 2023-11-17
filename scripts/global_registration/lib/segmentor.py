import numpy as np
import math
import open3d as o3d
from lib.visualizer import Visualizer
from icecream import ic
import copy


def count_points_on_line(matrix, points, tolerance=105):
    x1, y1 = points[0]
    x2, y2 = points[1]

    # Calculate the slope (m) and y-intercept (b) of the line
    if x2 - x1 != 0:
        slope = (y2 - y1) / (x2 - x1)
        intercept = y1 - slope * x1
    else:
        # Vertical line, handle separately to avoid division by zero
        slope = float('inf')
        intercept = x1

    # Sort the points to make sure x1 <= x2
    x_min, x_max = min(x1, x2), max(x1, x2)
    y_min, y_max = min(y1, y2), max(y1, y2)

    # Count the points on the line that are exactly between x1 and x2
    count = 0
    for point in matrix:
        x, y = point
        # Check if the point is on the line within the given tolerance
        if abs(y - (slope * x + intercept)) < tolerance and (x_min <= x <= x_max or y_min <= y <= y_max):
            count += 1

    return count
    
def get_indices(rectangle):
    sorted_indices = np.argsort(rectangle[:, 1])
    i_upper = sorted_indices[2:]
    i_lower = sorted_indices[:2]
    
    sorted_indices = np.argsort(rectangle[:, 0])
    i_left = sorted_indices[:2]
    i_right = sorted_indices[2:]

    return i_upper, i_lower, i_left, i_right


def grow_upper(rectangle, matrix, indices, angle, step):
    counter = 0
    while True:
        add_matrix = np.zeros((4, 2))
        
        add_matrix[indices, 1] += step
        add_matrix = rotate(add_matrix, angle)

        rectangle += add_matrix
        distance = abs(math.dist(rectangle[indices][0], rectangle[indices][1]))
        counter += count_points_on_line(matrix, rectangle[indices])
        if  counter > distance/10:
            break
        
    return rectangle

def grow_lower(rectangle, matrix, indices, angle, step):
    counter = 0
    while True:
        add_matrix = np.zeros((4, 2))
        
        add_matrix[indices, 1] -= step
        add_matrix = rotate(add_matrix, angle)

        rectangle += add_matrix
        distance = abs(math.dist(rectangle[indices][0], rectangle[indices][1]))
        counter += count_points_on_line(matrix, rectangle[indices])
        if  counter > distance/10:
            break
        
    return rectangle

def grow_left(rectangle, matrix, indices, angle, step):
    
    while True:
        add_matrix = np.zeros((4, 2))
        
        add_matrix[indices, 0] -= step
        add_matrix = rotate(add_matrix, angle)

        rectangle += add_matrix
        distance = abs(math.dist(rectangle[indices][0], rectangle[indices][1]))
        counter = count_points_on_line(matrix, rectangle[indices])
        if  counter > distance/100:
            ic(distance)
            break
        
    return rectangle

def grow_right(rectangle, matrix, indices, angle, step):
    
    while True:
        add_matrix = np.zeros((4, 2))
        
        sorted_indices = np.argsort(rectangle[:, 0])
        #horizontal_points = rectangle[sorted_indices[2:]]
        
        add_matrix[indices, 0] += step
        add_matrix = rotate(add_matrix, angle)

        rectangle += add_matrix
        distance = abs(math.dist(rectangle[indices][0], rectangle[indices][1]))
        counter = count_points_on_line(matrix, rectangle[indices])
        if  counter > distance/100:
            ic(distance)
            break
        
    return rectangle

def grow_rectangle(rectangle, matrix, angle, step=10):
    i_upper, i_lower, i_left, i_right = get_indices(rectangle)
    
    #grow_right(rectangle, matrix, i_right, angle, step)
    #Visualizer().draw_pointcloud_and_lines(matrix, rectangle)
    #grow_left(rectangle, matrix, i_left, angle, step)
    #Visualizer().draw_pointcloud_and_lines(matrix, rectangle)
    grow_upper(rectangle, matrix, i_upper, angle, step)
    
    grow_lower(rectangle, matrix, i_lower, angle, step)
   
    grow_right(rectangle, matrix, i_right, angle, step)
    
    grow_left(rectangle, matrix, i_left, angle, step)
    Visualizer().draw_pointcloud_and_lines(matrix, rectangle)
    
        
    return rectangle
    
def translate(rectangle, x, y):
    for point in rectangle:
        point[0] += x
        point[1] += y
    return rectangle
    

def rotate(rectangle, angle):
    # Define the rotation matrix
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                [np.sin(angle), np.cos(angle)]])
    
    return np.dot(rectangle, rotation_matrix)
    

class Segmentor:
    def __init__(self, floorModel, points) -> None:
        self.floorModel = floorModel
        self.points = points
        
        
    def get_angle(self):
        sorted_indices = np.argsort(self.points[:, 1])
        
        smallest_y_points = self.points[sorted_indices[:2]]
        angle_rad = np.arctan2(smallest_y_points[0][1] - smallest_y_points[1][1], smallest_y_points[0][0] - smallest_y_points[1][0])
        angle_deg = np.degrees(angle_rad)
        return angle_rad
    
    
    def prepare_matrix(self, pcd):
        pcd_matrix = np.asarray(pcd)
        pcd_matrix= np.delete(pcd_matrix.astype(np.float32), 1, axis=1)
        
        return pcd_matrix
    
    def find_wall(self):
        angle = self.get_angle()
        ic(np.cos(np.pi/2))
        floorModel_matrix = self.prepare_matrix(self.floorModel.points)
        
        #initialize rectangle
        rectangle = np.array([[0, 0], [200, 0], [200, 200], [0, 200]])
        rectangle = translate(rectangle, 7000, 1000)
        rectangle_rotated = rotate(rectangle, -angle)
        Visualizer().draw_pointcloud_and_lines(floorModel_matrix, rectangle_rotated)
        
        rectangle = grow_rectangle(rectangle_rotated, floorModel_matrix, -angle)
    
        
        