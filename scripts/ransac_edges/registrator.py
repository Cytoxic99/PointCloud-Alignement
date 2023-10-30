import open3d as o3d
from icecream import ic
from visualizer import Visualizer
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.stats import mode
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt

class Registrate:
    
    def __init__(self,source_down, target_down, voxel_size, clusters=None) -> None:
        self.source_down = source_down
        self.target_down = target_down

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
            
        
    def refine_registration(self, source, target, transformation, voxel_size):
        distance_threshold = voxel_size * 0.4
        
        result = o3d.pipelines.registration.registration_icp(
            source, target, distance_threshold, transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        return result
    
    def get_best_cluster(self):
        best_clusters = []
        best_loss = 10000000000000
        for cluster in self.clusters:
            loss = (ic(len(cluster.points)) - ic(len(self.source_down.points)))**2
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
        source_plane = self.get_plane(self.source_down, 10)
        target_plane = self.get_plane(self.target_down, 10)
        radius_normal = 10
         
         # Extract the ceiling or floor from the scan
        #source_plane.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        #target_plane.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        
        
        # Get the normal vectors
        #source_normals = np.asarray(source_plane.normals)
        #target_normals = np.asarray(target_plane.normals)
        
        #o3d.visualization.draw_geometries([source_plane, target_plane], point_show_normal=True)
        
        # Take the median from all the normalvectors
        #source_normal, _ = mode(source_normals, axis=0)
        #target_normal, _ = mode(target_normals, axis=0)
        
        source_normal = self.compute_normal(source_plane)
        target_normal = self.compute_normal(target_plane)
        
        ic(source_normal, target_normal)
        
        # Reshape the vectors, so that open3d can work with it
        source_normal=np.reshape(source_normal, (1, -1))
        target_normal=np.reshape(target_normal, (1, -1))
        
        #Scikit-function to align vectors
        rotation_matrix, _ = R.align_vectors(source_normal, target_normal)
        rotation_matrix = rotation_matrix.as_matrix()
        
        rotation_matrix_4x4 = np.eye(4)
        rotation_matrix_4x4[:3, :3] = rotation_matrix
        rotation_matrix_4x4[3, 3] = 1

        Visualizer().draw_registration_result(source_plane, target_plane, rotation_matrix_4x4)
        
        return rotation_matrix_4x4
    
    
    
    