import open3d as o3d
from icecream import ic
from visualizer import Visualizer
import numpy as np
from scipy.spatial.transform import Rotation as R

class Registrate:
    
    def __init__(self,source_down, target_down, source_fpfh, target_fpfh, voxel_size, clusters=None) -> None:
        self.source_down = source_down
        self.target_down = target_down
        self.source_fpfh = source_fpfh
        self.target_fpfh = target_fpfh
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
            
    def execute_global_registration(self):
        distance_threshold = 0.001
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            self.source_down, self.target_down, self.source_fpfh, self.target_fpfh, True,
            distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            3, [
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                    0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                    distance_threshold)
            ], o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 0.999))
        
        self.transformation_global_registration = result.transformation
        
        return result
    
    def execute_fast_global_registration(self):
        distance_threshold = self.voxel_size * 0.5
        
        result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
            self.source_down, self.target_down, self.source_fpfh, self.target_fpfh,
            o3d.pipelines.registration.FastGlobalRegistrationOption(
                maximum_correspondence_distance=distance_threshold))
        
        self.transformation_global_registration = result.transformation
        return result
        
    def refine_registration(self, source, target, source_fpfh, target_fpfh, transformation, voxel_size):
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
        
    def registrate_rotation(self):
        source_plane = self.get_plane(self.source_down, 10)
        target_plane = self.get_plane(self.target_down, 10)
        radius_normal = self.voxel_size*2
        
        
        
        source_plane.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        target_plane.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        
        o3d.visualization.draw_geometries([source_plane, target_plane])
        
        # Get the normal vectors
        source_normals = np.asarray(source_plane.normals)
        target_normals = np.asarray(target_plane.normals)
        
        source_normal = np.median(source_normals, axis=0)
        target_normal = np.median(target_normals, axis=0)
        
        source_normal=np.reshape(source_normal, (1, -1))
        target_normal=np.reshape(target_normal, (1, -1))
        
        rotation_matrix, _ = R.align_vectors(source_normal, target_normal)
        ic(rotation_matrix.as_matrix())
        
        return rotation_matrix.as_matrix()
    
    
    
    