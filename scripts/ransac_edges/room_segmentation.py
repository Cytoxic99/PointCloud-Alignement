import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import distance
import copy
import random
from visualizer import Visualizer
from icecream import ic

class RoomFinder():
    
    def __init__(self, source, target) -> None:
        self.source = source
        self.target = target
    
    def generateTransformation(self):
        # Generate random translations in the x and y directions
        x_translation = np.random.uniform(-1.0, 1000.0)  # Replace with your desired range
        y_translation = np.random.uniform(-1.0, 1000.0)  # Replace with your desired range
        rand_int = random.randrange(1, 360)
        rotation_angle =(2*np.pi)/rand_int

        # Create the transformation matrix for the random translation
        T = np.identity(4)
        T[0, 3] = x_translation
        T[2, 3] = y_translation
        T[:, 0] = np.array([np.cos(rotation_angle), 0, np.sin(rotation_angle), 0])
        T[:, 2] = np.array([-np.sin(rotation_angle), 0, np.cos(rotation_angle), 0])
        return T
        
    def pointsInside(self):
        # Create your volume of interest (e.g., an axis-aligned bounding box)
        min_bound, max_bound = self.source.get_min_bound(), self.source.get_max_bound()
        aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
        
        # Use crop function to extract the points inside the bounding box
        cropped_point_cloud = self.target.crop(aabb)

        # Get the number of points inside the volume
        count_inside_volume = len(cropped_point_cloud.points)

        return count_inside_volume
    
    def adjustCoord(self, pcd):
        # Access the point coordinates as a NumPy array
        points = np.asarray(pcd.points)

        if len(points) > 0:
            # Find the index of the points with the lowest coordinates
            min_x_index = np.argmin(points[:, 0])
            min_y_index = np.argmin(points[:, 1])
            min_z_index = np.argmin(points[:, 2])
            

            # Get the points with the lowest coordinates
            lowest_x_point = points[min_x_index][0]
            lowest_y_point = points[min_y_index][1]
            lowest_z_point = points[min_z_index][2]

            # Calculate the translation vector to move the lowest point to the origin
            translation_vector_x = -lowest_x_point
            translation_vector_y = -lowest_y_point
            translation_vector_z = -lowest_z_point

            # Create a translation matrix
            translation_matrix = np.identity(4)
            translation_matrix[0, 3] = translation_vector_x
            translation_matrix[1, 3] = translation_vector_y
            translation_matrix[2, 3] = translation_vector_z

            # Transform the original point cloud using the translation matrix
            translated_point_cloud = pcd.transform(translation_matrix)
            return translated_point_cloud
    
    def getPosition(self):
        visualizer = Visualizer()
        self.source = self.adjustCoord(self.source)
        self.target = self.adjustCoord(self.target)
        #visualizer.draw_registration_result(self.source, self.target, np.eye(4))
        n_iterations = 1000
        best_num_points = 100000000000
        best_transform = None
        
        for i in range(n_iterations):
            transformation = self.generateTransformation()
            self.source.transform(transformation)
            curr_num_points = self.pointsInside()
            if curr_num_points < best_num_points:
                best_num_points = curr_num_points
                best_transform = transformation
                visualizer.draw_registration_result(self.source, self.target, best_transform)
        
        
        return best_transform
            