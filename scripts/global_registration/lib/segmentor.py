import numpy as np
import math
import open3d as o3d
from lib.visualizer import Visualizer
from icecream import ic
import copy
import threading
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

def split_bounding_box(pointcloud):
    # Extract the bounding box coordinates
    min_x = min(point[0] for point in pointcloud)
    min_y = min(point[1] for point in pointcloud)
    max_x = max(point[0] for point in pointcloud)
    max_y = max(point[1] for point in pointcloud)

    # Define the number dof steps for x and y
    num_steps = 5

    # Calculate evenly spaced x and y values within the bounding box
    x_values = np.linspace(min_x, max_x, num_steps)
    y_values = np.linspace(min_y, max_y, num_steps)
    
    x_values = x_values[: -1]
    y_values = y_values[: -1]

    # Create a list of (x, y) coordinates representing the evenly spaced grid
    grid_coordinates = [(x, y) for x in x_values for y in y_values]

    return grid_coordinates

def is_rectangle_inside_pointcloud(rectangle_points, pointcloud_points):
    # Convert rectangle points to a Shapely Polygon
    rectangle_polygon = Polygon(rectangle_points)

    # Get the bounding box of the point cloud
    min_x = min(pointcloud_points[:, 0])
    min_y = min(pointcloud_points[:, 1])
    max_x = max(pointcloud_points[:, 0])
    max_y = max(pointcloud_points[:, 1])

    # Create a bounding box around the point cloud
    pointcloud_bounding_box = Polygon([(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)])

    # Check if all rectangle vertices are inside the point cloud bounding box
    if not rectangle_polygon.within(pointcloud_bounding_box):
        return False

    # If all points are inside the point cloud bounding box, return True
    return True


#Should return True if there is a certain density of points along the line reached
def line_reached(rectangle, indices, count):
    base = 1
    distance = abs(math.dist(rectangle[indices[0]], rectangle[indices[1]]))
    while not (int(distance/base) < 10 and int(distance/base) >= 1):
        base *= 10  
    if base < 101:
        return count > distance/base
    
    else:
        return count > distance/(base/10)


#checks all the line aagain when the rectangle is finished
def overall_checker(i_upper, i_lower, I_left, i_right):
    pass

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
    grown = False
    while True:
        counter = count_points_on_line(matrix, rectangle[indices])
        if  line_reached(rectangle, indices, counter):
            ic("Line reached!")
            break
        add_matrix = np.zeros((4, 2))
        
        add_matrix[indices, 1] += step
        add_matrix = rotate(add_matrix, angle)

        rectangle += add_matrix
        grown = True
        
    return grown

def grow_lower(rectangle, matrix, indices, angle, step):
    grown = False
    while True:
        counter = count_points_on_line(matrix, rectangle[indices])
        if  line_reached(rectangle, indices, counter):
            ic("Line reached!")
            break
        add_matrix = np.zeros((4, 2))
        
        add_matrix[indices, 1] -= step
        add_matrix = rotate(add_matrix, angle)

        rectangle += add_matrix
        grown = True
        
        
        
    return grown

def grow_left(rectangle, matrix, indices, angle, step):
    grown = False
    while True:
        counter = count_points_on_line(matrix, rectangle[indices])
        if  line_reached(rectangle, indices, counter):
            ic("Line reached!")
            break
        
        add_matrix = np.zeros((4, 2))
        
        add_matrix[indices, 0] -= step
        add_matrix = rotate(add_matrix, angle)

        rectangle += add_matrix
        grown = True
        
        
    return grown

def grow_right(rectangle, matrix, indices, angle, step):
    grown = False
    while True:
        counter = count_points_on_line(matrix, rectangle[indices])
        if  line_reached(rectangle, indices, counter):
            ic("Line reached!")
            break
        
        add_matrix = np.zeros((4, 2))
        
        add_matrix[indices, 0] += step
        add_matrix = rotate(add_matrix, angle)

        rectangle += add_matrix
        grown = True
        
        
    return grown

def grow_rectangle(rectangle, matrix, angle, step=10):
    i_upper, i_lower, i_left, i_right = get_indices(rectangle)
    
    while True:
        grown = False
        if grow_upper(rectangle, matrix, i_upper, angle, step):
            grown = True
    
        if grow_lower(rectangle, matrix, i_lower, angle, step):
            grown = True
    
        if grow_right(rectangle, matrix, i_right, angle, step):
            grown = True
    
        if grow_left(rectangle, matrix, i_left, angle, step):
            grown = True
            
        if not grown:
            break

    
        
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

def is_point_in_rectangle(matrix, rectangle):
    # Convert rectangle points to a Shapely Polygon
    rectangle_polygon = Polygon(rectangle)

    # Initialize a counter for points inside the rectangle
    points_inside_count = 0

    # Iterate through each point in the matrix
    for row in matrix:
        point = Point(row[0], row[1])
        if rectangle_polygon.contains(point):
            return True

    return False

def merge_intersecting_rectangles(rectangles):
    # Convert rectangle points to Shapely Polygons
    rectangles_polygons = [Polygon(rectangle) for rectangle in rectangles]

    # Use unary_union to merge intersecting polygons
    merged_polygon = unary_union(rectangles_polygons)
    
    edge_points = []
    # Iterate through each polygon in the MultiPolygon
    for polygon in merged_polygon.geoms:
        # Get the exterior (outer ring) of the polygon
        exterior = polygon.exterior

        # Extract the coordinates of the exterior ring
        exterior_coords = list(exterior.coords)

        # Append the coordinates to the list of edge points
        edge_points.extend(exterior_coords)

    return edge_points
    
    

class Segmentor:
    def __init__(self, floorModel, points) -> None:
        self.floorModel = floorModel
        self.points = points
        
        
    def is_rectangle_inside_pointcloud(self, rectangle_points, pointcloud_points):
        # Convert rectangle points to a Shapely Polygon
        rectangle_polygon = Polygon(rectangle_points)

        # Create a bounding box around the point cloud
        pointcloud_bounding_box = Polygon(self.points)

        # Check if all rectangle vertices are inside the point cloud bounding box
        if not rectangle_polygon.within(pointcloud_bounding_box):
            return False

        # If all points are inside the point cloud bounding box, return True
        return True
        
        
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
    
    def find_rooms(self, rectangle, rectangles, x, y, rectangles_init):
        angle = self.get_angle()
        floorModel_matrix = self.prepare_matrix(self.floorModel.points)
        
        rectangle = translate(rectangle, x, y)
        rectangle_rotated = rotate(rectangle, -angle)
        if self.is_rectangle_inside_pointcloud(rectangle_rotated, floorModel_matrix) and not is_point_in_rectangle(floorModel_matrix, rectangle_rotated):
            rectangles_init.append(copy.deepcopy(rectangle_rotated))
        
            rectangle = grow_rectangle(rectangle_rotated, floorModel_matrix, -angle)

            rectangles.append(rectangle)
            
        
        else:
            ic("not inside!")
        
        
    def start(self):
        visualizer = Visualizer()
        rectangles = []
        rectangles_init = []
        #initialize rectangle
        rectangle = np.array([[0, 0], [200, 0], [200, 200], [0, 200]])
        
        floorModel_matrix = self.prepare_matrix(self.floorModel.points)
        
        # Use the split_bounding_box function to generate grid coordinates
        grid_coordinates = split_bounding_box(floorModel_matrix)

        threads = []
        for x, y in grid_coordinates:
            thread = threading.Thread(target=self.find_rooms, args=(copy.deepcopy(rectangle), rectangles, x, y, rectangles_init))
            threads.append(thread)
            
        # Start all threads
        for thread in threads:
            thread.start()

        # Wait for all threads to finish
        for thread in threads:
            thread.join()
            
        return rectangles
            
        
        
        Visualizer().draw_pointcloud_and_rectangles(floorModel_matrix, rectangles_init)
        Visualizer().draw_pointcloud_and_rectangles(floorModel_matrix, rectangles)
        
        merged_rectangles = merge_intersecting_rectangles(rectangles)
        Visualizer().draw_pointcloud_and_rectangles(floorModel_matrix, merged_rectangles)