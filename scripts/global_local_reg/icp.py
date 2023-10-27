import open3d as o3d
import numpy as np
from icecream import ic

class icpCalc:
    
    def __init__(self, movFile, fixFile) -> None:
        self.model_path = movFile
        self.room_scan_path = fixFile
        
    # Perform ICP registration with point-to-plane
    def icp_point_to_plane(self, max_distance=0.02, max_iterations=10):

        #Load the point clouds
        model_pc = o3d.io.read_point_cloud(self.model_path)
        room_scan_pc = o3d.io.read_point_cloud(self.room_scan_path)

        # Compute normals for both PointClouds
        model_pc.estimate_normals()
        room_scan_pc.estimate_normals()

        # Create the KD-Tree for fast correspondence finding
        source_tree = o3d.geometry.KDTreeFlann(model_pc)

        # Initialize the transformation matrix
        transformation = np.identity(4)

        ic("Reached algorithm")
        for i in range(max_iterations):
            ic(i)
            # Compute normals for the target PointCloud
            model_pc.estimate_normals()
            room_scan_pc.estimate_normals()
            
            # Find correspondences
            correspondences = []

            for point in room_scan_pc.points:
                _, idx, _ = source_tree.search_knn_vector_3d(point, 1)
                source_point = model_pc.points[idx[0]]
                correspondences.append((source_point, point))
            ic(correspondences)

            # Convert correspondences to numpy arrays
            source_points = np.array([corr[0] for corr in correspondences])
            target_points = np.array([corr[1] for corr in correspondences])

            source_points=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(source_points))
            target_points=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(target_points))

            source_points.estimate_normals()
            target_points.estimate_normals()

            # Compute the transformation
            transformation = o3d.pipelines.registration.registration_icp(
                source=source_points,
                target=target_points,
                max_correspondence_distance=max_distance,
                init=transformation,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            ).transformation
            

        return transformation









