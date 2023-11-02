import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import distance
import copy
import random
from visualizer import Visualizer
from icecream import ic

class RoomFinder():
    
    def __init__(self, floorModel, roomScan) -> None:
        self.floorModel = floorModel
        self.roomScan = roomScan
        
        self.x_min_roomScan = 0
        self.y_min_roomScan = 0
        self.z_min_roomScan = 0
        self.x_max_roomScan = 0
        self.y_max_roomScan = 0
        self.z_max_roomScan = 0
        
        self.x_min_floorModel = 0
        self.y_min_floorModel = 0
        self.z_min_floorModel = 0
    
    def findBorders(self):
        points_floorModel = np.asarray(self.floorModel.points)
        # Find the index of the points with the lowest and heighest coordinates
        min_x_index = np.argmin(points_floorModel[:, 0])
        min_y_index = np.argmin(points_floorModel[:, 1])
        min_z_index = np.argmin(points_floorModel[:, 2])
        max_x_index = np.argmax(points_floorModel[:, 0])
        max_y_index = np.argmax(points_floorModel[:, 1])
        max_z_index = np.argmax(points_floorModel[:, 2])

        # Get the points with the lowest and heighest coordinates
        self.x_min_floorModel = points_floorModel[min_x_index][0]
        self.y_min_floorModel = points_floorModel[min_y_index][1]
        self.z_min_floorModel = points_floorModel[min_z_index][2]
        self.x_max_floorModel = points_floorModel[max_x_index][0]
        self.y_max_floorModel = points_floorModel[max_y_index][1]
        self.z_max_floorModel = points_floorModel[max_z_index][2]
        
        points_roomScan = np.asarray(self.roomScan.points)
        # Find the index of the points with the lowest and heighest coordinates
        min_x_index = np.argmin(points_roomScan[:, 0])
        min_y_index = np.argmin(points_roomScan[:, 1])
        min_z_index = np.argmin(points_roomScan[:, 2])
        max_x_index = np.argmax(points_roomScan[:, 0])
        max_y_index = np.argmax(points_roomScan[:, 1])
        max_z_index = np.argmax(points_roomScan[:, 2])

        # Get the points with the lowest and heighest coordinates
        self.x_min_roomScan = points_roomScan[min_x_index][0]
        self.y_min_roomScan = points_roomScan[min_y_index][1]
        self.z_min_roomScan = points_roomScan[min_z_index][2]
        self.x_max_roomScan = points_roomScan[max_x_index][0]
        self.y_max_roomScan = points_roomScan[max_y_index][1]
        self.z_max_roomScan = points_roomScan[max_z_index][2]
    
    def is_inside(self, T):
        # copy of roomscan
        pcd_copy = copy.deepcopy(self.roomScan)
        pcd_copy.transform(T)
        
        # Create your volume of interest (e.g., an axis-aligned bounding box)
        min_bound, max_bound = self.floorModel.get_min_bound(), self.floorModel.get_max_bound()
        aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
        
        # Use crop function to extract the points inside the bounding box
        cropped_point_cloud = pcd_copy.crop(aabb)

        # Get the number of points inside the volume
        count_inside_volume = len(cropped_point_cloud.points)
        
        if count_inside_volume == len(self.roomScan.points):
            return True
        else:
            return False
            
    
    def generateTransformation(self):
        i = 0
        while True:
            # Generate random translations in the x and y directions
            x_translation = np.random.uniform(-1000, 1000)
            z_translation = np.random.uniform(-1000, 1000)
            
            rotation_random = random.randrange(1, 360)
            rotation_angle = (2*np.pi) / rotation_random

            # Create the transformation matrix for the random translation
            T = np.identity(4)
            T[0, 3] = x_translation
            T[2, 3] = z_translation
            T[:, 0] = np.array([np.cos(rotation_angle), 0, np.sin(rotation_angle), 0])
            T[:, 2] = np.array([-np.sin(rotation_angle), 0, np.cos(rotation_angle), 0])
            
            if self.is_inside(T):
                break
            
                    
        return T
        
    def pointsInside(self, roomScan_copy):
        # Create your volume of interest (e.g., an axis-aligned bounding box)
        min_bound, max_bound = roomScan_copy.get_min_bound(), roomScan_copy.get_max_bound()
        aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
        # Use crop function to extract the points inside the bounding box
        cropped_point_cloud = self.floorModel.crop(aabb)

        # Get the number of points inside the volume
        count_inside_volume = len(cropped_point_cloud.points)

        return count_inside_volume
    
    def adjustCoord(self, pcd, floorModel):
        # Access the point coordinates as a NumPy array
        points = np.asarray(pcd.points)

        if len(points) > 0:
            # Find the index of the points with the lowest coordinates
            min_x_index = np.argmin(points[:, 0])
            min_y_index = np.argmin(points[:, 1])
            min_z_index = np.argmin(points[:, 2])
            

            # Get the points with the lowest and heighest coordinates
            lowest_x_point = points[min_x_index][0]
            lowest_y_point = points[min_y_index][1]
            lowest_z_point = points[min_z_index][2]

            # Calculate the translation vector to move the lowest point to the origin
            translation_vector_x = -lowest_x_point
            translation_vector_y = -lowest_y_point
            if floorModel:
                translation_vector_y -= 300
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
        
        self.floorModel = self.adjustCoord(self.floorModel, True)
        self.roomScan = self.adjustCoord(self.roomScan, False)
        self.findBorders()
        
        
        #visualizer.draw_registration_result(self.floorModel, self.roomScan, np.eye(4))
        n_iterations = 1000
        best_num_points = 100000000000
        best_transform = None
        
        for i in range(n_iterations):
            transformation = self.generateTransformation()
            roomScan_copy = copy.copy(self.roomScan)  # Create a copy of the original roomscan
            roomScan_copy.transform(transformation)  # Apply the transformation to the copy
            curr_num_points = self.pointsInside(roomScan_copy)  # Calculate the points inside the transformed floorModel
            if curr_num_points < best_num_points:
                best_num_points = curr_num_points
                best_transform = transformation
                ic(best_num_points)
                ic(best_transform)
        visualizer.draw_registration_result(self.floorModel, self.roomScan, best_transform)
        return best_transform
            