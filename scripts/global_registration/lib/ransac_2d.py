import numpy as np
import math
import open3d as o3d
from lib.visualizer import Visualizer
from icecream import ic
import copy

class Ransac:
    def __init__(self, floorModel, roomScan, points_floor, points_room) -> None:
        self.floorModel = floorModel
        self.roomScan = roomScan
        self.points_room = points_room
        self.points_floor = points_floor
        
    def prepare_matrix(self, pcd):
        pcd_matrix = np.asarray(pcd)
        pcd_matrix= np.delete(pcd_matrix.astype(np.float32), 1, axis=1)
        
        return pcd_matrix
    
    def transform_matrix(self, matrix):
        # Find the point with the minimum x and y coordinates in the point cloud
        min_matrix = matrix[np.argmin(np.sum(matrix, axis=1))]
        min_points = self.points_room[np.argmin(np.sum(self.points_room, axis=1))]
        
        translation = min_points - min_matrix
        
        transformed_matrix = matrix + translation
        
        return transformed_matrix

    def count_points(self, matrix):
        
        mask = (
            (matrix[:, 0] >= min(self.points_room[:, 0])) &
            (matrix[:, 0] <= max(self.points_room[:, 0])) &
            (matrix[:, 1] >= min(self.points_room[:, 1])) &
            (matrix[:, 1] <= max(self.points_room[:, 1]))
        )
        count = np.sum(mask)
        return count
    
    def get_angle(self):
        sorted_indices = np.argsort(self.points_room[:, 1])
        
        smallest_y_points = self.points_room[sorted_indices[:2]]
        angle_rad = np.arctan2(smallest_y_points[0][1] - smallest_y_points[1][1], smallest_y_points[0][0] - smallest_y_points[1][0])
        angle_deg = np.degrees(angle_rad)
        ic(angle_deg)
        return angle_rad
        
    
    def ransac(self, matrix):
        step_size = 500
        best_points = 10000000000000
        best_matrix = None
        angle = self.get_angle()
        
        current_matrix = copy.deepcopy(matrix)
        
        #Get the points with the smallest y values --> So the lower two points
        sorted_indices_room_x = np.argsort(self.points_room[:, 1])
        sorted_indices_floor_x = np.argsort(self.points_floor[:, 1])
        smallest_points_room_x = self.points_room[sorted_indices_room_x[:2]]
        smallest_points_floor_x = self.points_floor[sorted_indices_floor_x[:2]]
        
        #get the distance between the points
        distance_room_x = math.dist(smallest_points_room_x[0], smallest_points_room_x[1])
        distance_floor_x = math.dist(smallest_points_floor_x[0], smallest_points_floor_x[1])

        #Calculate the way, which the rectangle has to move in x direction
        max_trans_x = distance_floor_x - distance_room_x
        steps_x = abs(int(max_trans_x / step_size))
        
        #Get the points with the smallest y values --> So the lower two points
        sorted_indices_room_y = np.argsort(self.points_room[:, 0])
        sorted_indices_floor_y = np.argsort(self.points_room[:, 0])
        smallest_points_room_y = self.points_room[sorted_indices_room_y[:2]]
        smallest_points_floor_y = self.points_floor[sorted_indices_floor_y[:2]]
        
        
        #get the distance between the points
        distance_room_y = math.dist(smallest_points_room_y[0], smallest_points_room_y[1])
        distance_floor_y = math.dist(smallest_points_floor_y[0], smallest_points_floor_y[1])

        #Calculate the way, which the rectangle has to move in x direction
        max_trans_y = distance_floor_y - distance_room_y
        steps_y = abs(int(max_trans_y / step_size))
        
        
        
        for y in range(steps_y):
            current_matrix[:, 0] = matrix[:, 0] - y*step_size*np.cos((np.pi/2) + angle)
            current_matrix[:, 1] = matrix[:, 1] - y*step_size*np.sin((np.pi/2) + angle)
            
            for x in range(steps_x):
                
                current_matrix[:, 0] -= step_size*np.cos(angle)
                current_matrix[:, 1] -= step_size*np.sin(angle)
                
                Visualizer().draw_pointcloud_and_lines(current_matrix, self.points_room)
                current_points = self.count_points(current_matrix)
                
                if(current_points < best_points):
                    ic(current_points)
                    best_points = current_points
                
        
    
    

    def start(self):
        floorModel_matrix = self.prepare_matrix(self.floorModel.points)
        
        floorModel_matrix = self.transform_matrix(floorModel_matrix)
        optimal_translation = self.ransac(floorModel_matrix)
        
        
        
        
        