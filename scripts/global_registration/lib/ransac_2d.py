import numpy as np
import open3d as o3d
from lib.visualizer import Visualizer
from icecream import ic
import copy

class Ransac:
    def __init__(self, floorModel, roomScan, points) -> None:
        self.floorModel = floorModel
        self.roomScan = roomScan
        self.points = points
        
    def prepare_matrix(self, pcd):
        pcd_matrix = np.asarray(pcd)
        pcd_matrix= np.delete(pcd_matrix.astype(np.float32), 1, axis=1)
        
        return pcd_matrix
    
    def transform_matrix(self, matrix):
        # Find the point with the minimum x and y coordinates in the point cloud
        min_matrix = matrix[np.argmin(np.sum(matrix, axis=1))]
        min_points = self.points[np.argmin(np.sum(self.points, axis=1))]
        
        translation = min_points - min_matrix
        
        transformed_matrix = matrix + translation
        
        return transformed_matrix

    def count_points(self, matrix):
        
        mask = (
            (matrix[:, 0] >= min(self.points[:, 0])) &
            (matrix[:, 0] <= max(self.points[:, 0])) &
            (matrix[:, 1] >= min(self.points[:, 1])) &
            (matrix[:, 1] <= max(self.points[:, 1]))
        )
        count = np.sum(mask)
        return count
    
    def get_angle(self):
        sorted_indices = np.argsort(self.points[:, 1])
        
        smallest_y_points = self.points[sorted_indices[:2]]
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
        
        matrix_max_x = np.max(matrix[:, 0])
        points_max_x = np.max(self.points[:, 0])
        
        max_trans_x = abs((matrix_max_x - points_max_x)*np.cos(angle))
        steps_x = int(max_trans_x / step_size)
        
        matrix_max_y = np.max(matrix[:, 1])
        points_max_y = np.max(self.points[:, 1])
        
        max_trans_y = abs((matrix_max_y - points_max_y)*np.sin((np.pi/2) + angle))
        steps_y = int(max_trans_y / step_size)
        ic(steps_x, steps_y)
        
        for y in range(steps_y + 20):
            current_matrix[:, 0] = matrix[:, 0] - y*step_size*np.cos((np.pi/2) + angle)
            current_matrix[:, 1] = matrix[:, 1] - y*step_size*np.sin((np.pi/2) + angle)
            Visualizer().draw_pointcloud_and_lines(current_matrix, self.points)
            for x in range(steps_x):
                
                current_matrix[:, 0] -= step_size*np.cos(angle)
                current_matrix[:, 1] -= step_size*np.sin(angle)
            
                current_points = self.count_points(current_matrix)
                
                if(current_points < best_points):
                    ic(current_points)
                    best_points = current_points
                
        
    
    

    def start(self):
        floorModel_matrix = self.prepare_matrix(self.floorModel.points)
        
        floorModel_matrix = self.transform_matrix(floorModel_matrix)
        optimal_translation = self.ransac(floorModel_matrix)
        
        
        
        
        