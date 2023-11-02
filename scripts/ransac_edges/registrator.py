import open3d as o3d
from icecream import ic
from visualizer import Visualizer
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.stats import mode
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt

class Registrate:
    
    def __init__(self,floorModel_down, roomScan_down, voxel_size, clusters=None) -> None:
        self.floorModel_down = floorModel_down
        self.roomScan_down = roomScan_down

        self.voxel_size = voxel_size
        self.clusters = clusters
        
        self.transformation_global_registration = None
        self.transformation_local_registration = None
    
    @property
    def globalTransformation(self):
        return self.transformation_global_registration
    
    @property
    def localTransformation(self):
        return self.transformation_local_registration
            
        
    def refine_registration(self, floorModel, roomScan, transformation, voxel_size):
        distance_threshold = voxel_size * 0.4
        
        result = o3d.pipelines.registration.registration_icp(
            floorModel, roomScan, distance_threshold, transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        return result
    
    def get_best_cluster(self):
        best_clusters = []
        best_loss = 10000000000000
        for cluster in self.clusters:
            loss = (ic(len(cluster.points)) - ic(len(self.floorModel_down.points)))**2
            if loss < self.voxel_size**2:
                best_clusters.append(cluster)
        return best_clusters
    
    
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
    
    def compute_normal(self, pcd):
        # Perform PCA to find the plane orientation
        X = np.asarray(pcd.points)
        pca = PCA(n_components=3).fit(X)
        plane_normal = pca.components_
        ic(plane_normal)
        
        plane_normal = np.round(plane_normal, decimals=1)
        return plane_normal[2]

        
    def registrate_rotation(self):
        floorModel_plane = self.get_plane(self.floorModel_down, 10)
        roomScan_plane = self.get_plane(self.roomScan_down, 10)
        radius_normal = 10
         
         # Extract the ceiling or floor from the scan
        #floorModel_plane.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        #roomScan_plane.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        
        
        # Get the normal vectors
        #floorModel_normals = np.asarray(floorModel_plane.normals)
        #roomScan_normals = np.asarray(roomScan_plane.normals)
        
        #o3d.visualization.draw_geometries([floorModel_plane, roomScan_plane], point_show_normal=True)
        
        # Take the median from all the normalvectors
        #floorModel_normal, _ = mode(floorModel_normals, axis=0)
        #roomScan_normal, _ = mode(roomScan_normals, axis=0)
        
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
    
    
    
    