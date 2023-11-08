import open3d as o3d
from icecream import ic
from lib.visualizer import Visualizer
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.stats import mode
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt
import copy

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
            plane_model, inliers = pcd.segment_plane(100, ransac_n=3,num_iterations=10000)
            
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
        
    def findLine(self, pcd):
        
        pcd_copy = copy.copy(pcd)
        points = np.asarray(pcd_copy.points)

        # Find the index of the points with the smallest x-coordinates in the xz-plane
        min_x = np.min(points[:, 0])
        min_y = np.min(points[points[:, 0] == min_x, 1])

        min_x_indices = np.where((points[:, 0] == min_x) & (points[:, 1] == min_y))

        ic(min_x_indices)


        # Create a new point cloud with the points with the smallest x-coordinates in the xz-plane
        smallest_x_points = o3d.geometry.PointCloud()
        smallest_x_points.points = o3d.utility.Vector3dVector(points[min_x_indices])

        # Extract the minimum and maximum x and z coordinates
        min_x = np.min(smallest_x_points.points, axis=0)
        max_x = np.max(smallest_x_points.points, axis=0)

        # Create a line along the x-axis in the xz-plane using the minimum and maximum coordinates
        line = o3d.geometry.LineSet()
        line.points = o3d.utility.Vector3dVector(np.array([min_x, max_x]))
        line.lines = o3d.utility.Vector2iVector([[0, 1]])
        
        return line
    
    def procrustes(self, line1, line2):
        # Extract points from the two lines
        points1 = np.asarray(line1.points)
        points2 = np.asarray(line2.points)

        # Apply Procrustes analysis to find the optimal transformation matrix
        r = R.align_vectors(points2, points1)
        
        rotation_matrix = r[0].as_matrix()
        
        rotation_matrix_4x4 = np.eye(4)
        rotation_matrix_4x4[:3, :3] = rotation_matrix
        rotation_matrix_4x4[3, 3] = 1
        
        Visualizer().draw_registration_result(line1, line2, rotation_matrix_4x4)
        
        return rotation_matrix_4x4
    
    
    
    def registrate(self):
        rotation_matrix = self.registrate_rotation()
        self.floorModel_down.transform(rotation_matrix)
        
        adjust_matrix = self.adjustCoord()
        self.floorModel_down.transform(adjust_matrix)
        
        #floorModel_line = self.findLine(self.floorModel_down)
        #roomScan_line = self.findLine(self.roomScan_down)
        
        #registrtate_wall = self.procrustes(floorModel_line, roomScan_line)
        #self.floorModel_down.transform(registrtate_wall)
        
        return np.dot(rotation_matrix, adjust_matrix)
    
    
    
    
    
    