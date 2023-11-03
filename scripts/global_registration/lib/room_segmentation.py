import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import distance
import copy
import random
from lib.visualizer import Visualizer
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
        
        room_copy = copy.deepcopy(self.floorModel)
        
        # Create your volume of interest (e.g., an axis-aligned bounding box)
        aabb = room_copy.get_minimal_oriented_bounding_box()
        
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
            x_translation = np.random.uniform(-10000, 10000)
            z_translation = np.random.uniform(-10000, 10000)
            
            rotation_random = random.randrange(1, 36)
            #rotation_angle = np.pi*(21/36)
            rotation_angle = np.pi * (rotation_random / 36)
            

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
        aabb_max = roomScan_copy.get_minimal_oriented_bounding_box(robust=True)
        aabb_min = roomScan_copy.get_minimal_oriented_bounding_box(robust=False).scale(0.9, roomScan_copy.get_center())
        
        
        # Use crop function to extract the points inside the bounding box
        cropped_point_cloud_max = self.floorModel.crop(aabb_max)
        cropped_point_cloud_min = self.floorModel.crop(aabb_min)

        # Get the number of points inside the volume
        count_inside_volume_max = len(cropped_point_cloud_max.points)
        count_inside_volume_min = len(cropped_point_cloud_min.points)

        return (count_inside_volume_max - count_inside_volume_min)
    
    
    
    def getPosition(self):
        visualizer = Visualizer()
        self.findBorders()
        
        #visualizer.draw_registration_result(self.floorModel, self.roomScan, np.eye(4))
        n_iterations = 1000
        best_num_points = 0
        best_transform = None
        i = 0
        
        for i in range(n_iterations):
            transformation = self.generateTransformation()
            roomScan_copy = copy.copy(self.roomScan)  # Create a copy of the original roomscan
            roomScan_copy.transform(transformation)  # Apply the transformation to the copy
            curr_num_points = self.pointsInside(roomScan_copy)  # Calculate the points inside the transformed floorModel
            if curr_num_points > best_num_points:
                best_num_points = curr_num_points
                best_transform = transformation
                ic(best_num_points)
                ic(best_transform)
            ic(i)
        
        return best_transform
            