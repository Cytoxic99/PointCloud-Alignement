import open3d as o3d
from icecream import ic
from lib.visualizer import Visualizer
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.stats import mode
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt
import copy
from shapely.geometry import MultiPoint, Polygon
import cv2

class Registrate:
    
    def __init__(self,floorModel_down, roomScan_down, voxel_size) -> None:
        self.floorModel_down = floorModel_down
        self.roomScan_down = roomScan_down

        self.voxel_size = voxel_size
        
    def compute_normal(self, pcd):
        # Perform PCA to find the plane orientation
        X = np.asarray(pcd.points)
        pca = PCA(n_components=3).fit(X)
        plane_normal = pca.components_
        
        plane_normal = np.round(plane_normal, decimals=1)
        return plane_normal[2]
        
    def get_plane(self, pcd, max_iterations):
        
        best_inliers = None
        for _ in range(max_iterations):
            # Plane model segmentation with RANSAC
            plane_model, inliers = pcd.segment_plane(100, ransac_n=4,num_iterations=10000)
            
            if best_inliers == None:
                best_inliers = inliers
            
            # Check if this model has more inliers than the previous best
            if len(inliers) > len(best_inliers):
                best_inliers = inliers
            
        inlier_cloud = pcd.select_by_index(best_inliers, invert=False)
        pcd = inlier_cloud
        #o3d.visualization.draw_geometries([inlier_cloud])
        return inlier_cloud

        
    def registrate_rotation(self):
        floorModel_plane = self.get_plane(self.floorModel_down, 10)
        roomScan_plane = self.get_plane(self.roomScan_down, 10)
        
        floorModel_normal = self.compute_normal(floorModel_plane)
        roomScan_normal = self.compute_normal(roomScan_plane)
        
        ic(floorModel_normal, roomScan_normal)
        
        # Reshape the vectors, so that open3d can work with it
        floorModel_normal=np.reshape(floorModel_normal, (1, -1))
        roomScan_normal=np.reshape(roomScan_normal, (1, -1))
        
        #Scikit-function to align vectors
        rotation_matrix, _ = R.align_vectors(floorModel_normal, roomScan_normal)
        rotation_matrix = rotation_matrix.as_matrix()
        
        rotation_matrix_4x4 = np.eye(4)
        rotation_matrix_4x4[:3, :3] = rotation_matrix
        rotation_matrix_4x4[3, 3] = 1
        
        return rotation_matrix_4x4
        
    def align_rotation(self):
        floor_box = self.floorModel_down.get_oriented_bounding_box()
        floor_rot = floor_box.R

        
        room_box = self.roomScan_down.get_oriented_bounding_box()
        room_rot = room_box.R
        
        T, _, _, _ = np.linalg.lstsq(room_rot, floor_rot, rcond=None)

        self.floorModel_down.rotate(T)
        
    
    def adjustCoord(self):
        # Access the point coordinates as a NumPy array
        points_floor = np.asarray(self.floorModel_down.points)
        points_room = np.asarray(self.roomScan_down.points)

        if len(points_floor) > 0 and len(points_room) > 0:
            # Find the index of the points with the lowest coordinates for both point clouds
            min_x_index_floor = np.argmin(points_floor[:, 0])
            min_y_index_floor = np.argmin(points_floor[:, 1])
            min_z_index_floor = np.argmin(points_floor[:, 2])
            
            min_x_index_room = np.argmin(points_room[:, 0])
            min_y_index_room = np.argmin(points_room[:, 1])
            min_z_index_room = np.argmin(points_room[:, 2])

            # Get the points with the lowest coordinates for both point clouds
            lowest_x_point_floor = points_floor[min_x_index_floor][0]
            lowest_y_point_floor = points_floor[min_y_index_floor][1]
            lowest_z_point_floor = points_floor[min_z_index_floor][2]
            
            lowest_x_point_room = points_room[min_x_index_room][0]
            lowest_y_point_room = points_room[min_y_index_room][1]
            lowest_z_point_room = points_room[min_z_index_room][2]

            # Calculate the translation vector to move the lowest point of floorModel to the origin
            translation_vector_x = -lowest_x_point_floor + lowest_x_point_room
            translation_vector_y = -lowest_y_point_floor + lowest_y_point_room
            translation_vector_z = -lowest_z_point_floor + lowest_z_point_room

            # Create a translation matrix
            translation_matrix = np.identity(4)
            translation_matrix[0, 3] = translation_vector_x
            translation_matrix[1, 3] = translation_vector_y
            translation_matrix[2, 3] = translation_vector_z
            
            # Reset the rotation of the point cloud to 0
            rotation_matrix = np.identity(4)
            
            # Combine the translation and rotation matrices
            result_matrix = np.dot(translation_matrix, rotation_matrix)
            
            return result_matrix
    
    def make_2d(self, pcd):
        pcd_copy = copy.deepcopy(pcd)
        projected_points = np.asarray(pcd_copy.points)
        projected_points[:, 1] = 0
        pcd_copy.points = o3d.utility.Vector3dVector(projected_points)
        
        return pcd_copy
    
    def rotate(self, pcd, angle_degrees):
        # Convert angle to radians
        angle_radians = np.radians(angle_degrees)
        
        # Create rotation matrix for rotation around y-axis
        rotation_matrix = np.array([[np.cos(angle_radians), 0, np.sin(angle_radians)],
                                    [0, 1, 0],
                                    [-np.sin(angle_radians), 0, np.cos(angle_radians)]])
        
        # Apply rotation to the point cloud
        pcd.rotate(rotation_matrix, center=(0, 0, 0))
        
        return rotation_matrix
    

    def get_rect(self, pcd):
        matrix = np.asarray(pcd.points)
        matrix_2d = np.delete(matrix.astype(np.float32), 1, axis=1)
        
        rect = cv2.minAreaRect(matrix_2d)
        
        points = cv2.boxPoints(rect)
        
        return points, rect[2]
        
    
    
    def registrate(self):
        
        
        rotation_matrix = self.registrate_rotation()
        self.floorModel_down.transform(rotation_matrix)
        
        adjust_matrix = self.adjustCoord()
        self.floorModel_down.transform(adjust_matrix)
        
        T = np.dot(rotation_matrix, adjust_matrix)
        
        floorModel_2d = self.make_2d(self.floorModel_down)
        roomScan_2d = self.make_2d(self.roomScan_down)
        
        points_room , room_rot = self.get_rect(roomScan_2d)
        _ , floor_rot = self.get_rect(floorModel_2d)
        abs_rot = floor_rot - room_rot
        
        self.rotate(self.floorModel_down, abs_rot)
        adjust_rot_matrix = self.rotate(floorModel_2d, abs_rot)
        points_floor , _ = self.get_rect(floorModel_2d)
        
        return T, floorModel_2d, roomScan_2d, points_floor, points_room
    
    
    
    
    
    
