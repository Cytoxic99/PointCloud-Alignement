from scipy.spatial.transform import Rotation
import numpy as np
import copy
from lib.visualizer import Visualizer
from icecream import ic
import open3d as o3d


def get_rotation_matrices():
    rotations = []
    # Define the angles of rotation in degrees
    angles_y = [0, 180]  # Rotation around y-axis
    angles_x = [0, 180]  # Rotation around x-axis

    # Generate all possible combinations of angles
    for angle_y in angles_y:
        for angle_x in angles_x:
            # Convert angles to radians
            angle_y_rad = np.radians(angle_y)
            angle_x_rad = np.radians(angle_x)
            
            # Create rotation matrices for y-axis and x-axis rotations
            rotation_y = Rotation.from_euler('y', angle_y_rad, degrees=False)
            rotation_x = Rotation.from_euler('x', angle_x_rad, degrees=False)
            
            # Combine the two rotations
            combined_rotation_matrix = rotation_y.as_matrix() @ rotation_x.as_matrix()

            rotations.append(combined_rotation_matrix)
            
    return rotations

def apply_rotation_2d(floorModel_2d, rectangle, matrix, angle):

    rectangle = np.insert(rectangle, 1, 0, axis=1)
    center = np.mean(rectangle, axis=0)
    rectangle = np.delete(rectangle, 1, axis=1)
    
    R_origin = Rotation.from_euler('y', angle, degrees=False).as_matrix()
    R_real = Rotation.from_euler('y', -angle, degrees=False).as_matrix()
    
    R = np.eye(4)
    R[:3, :3] = matrix
    
    T_origin = np.eye(4)
    T_origin[:3, 3] = -center
    
    T_real = np.eye(4)
    T_real[:3, 3] = center
    
    floorModel_matrix = np.asarray(floorModel_2d.points)
    
    floorModel_matrix[:,:] -= center
    floorModel_matrix = np.dot(floorModel_matrix, R_origin.T)
    floorModel_matrix = np.dot(floorModel_matrix, matrix.T)
    floorModel_matrix = np.dot(floorModel_matrix, R_real.T)
    floorModel_matrix[:,:] += center
    
    # Insert zeros at the second column
    floorModel_matrix = np.delete(floorModel_matrix, 1, axis=1)
    
    #Visualizer().draw_pointcloud_and_lines(floorModel_matrix, rectangle)
    
def apply_rotation_3d(floorModel, roomScan, rectangle, matrix, angle):

    rectangle = np.insert(rectangle, 1, 0, axis=1)
    center = np.mean(rectangle, axis=0)
    rectangle = np.delete(rectangle, 1, axis=1)
    
    R_origin = Rotation.from_euler('y', angle, degrees=False).as_matrix()
    R_real = Rotation.from_euler('y', -angle, degrees=False).as_matrix()
    
    R = np.eye(4)
    R[:3, :3] = matrix
    
    T_origin = np.eye(4)
    T_origin[:3, 3] = -center
    
    T_real = np.eye(4)
    T_real[:3, 3] = center
    
    floorModel_matrix = np.asarray(floorModel.points)
    
    floorModel_matrix[:,:] -= center
    floorModel_matrix = np.dot(floorModel_matrix, R_origin.T)
    floorModel_matrix = np.dot(floorModel_matrix, matrix.T)
    floorModel_matrix = np.dot(floorModel_matrix, R_real.T)
    floorModel_matrix[:,:] += center
    
    # Set the points of the PointCloud using the NumPy array
    floorModel.points = o3d.utility.Vector3dVector(floorModel_matrix)
    dists = floorModel.compute_point_cloud_distance(roomScan)
    dists = np.asarray(dists)
    dists = np.mean(dists)
    ic(dists)
    
    Visualizer().draw_multiple_pointclouds([floorModel, roomScan])
    return dists
    
    

class Local:
    
    def __init__(self, floorModel, roomScan, floorModel_2d, roomScan_2d, rectangle, points_room, angle) -> None:
        self.floorModel = floorModel
        self.roomScan = roomScan
        self.floorModel_2d = floorModel_2d
        self.roomScan_2d = roomScan_2d
        self.rectangle = rectangle
        self.points_room = points_room
        self.angle = angle
        
    def registrate(self, matrices):
        for matrix in matrices:
            apply_rotation_2d(copy.deepcopy(self.floorModel_2d), copy.deepcopy(self.rectangle), matrix, self.angle)
            apply_rotation_3d(copy.deepcopy(self.floorModel), copy.deepcopy(self.roomScan), copy.deepcopy(self.rectangle), matrix, self.angle)
            
            
            
    def start(self):
        
        matrices = get_rotation_matrices()
        self.registrate(matrices)
        
        