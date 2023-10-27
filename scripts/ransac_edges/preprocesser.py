import open3d as o3d

class Preprocessor:
    
    def __init__(self, point_cloud, voxel_size) -> None:
        self.pcd =point_cloud
        self.voxel_size = voxel_size
    
    def preprocess_point_cloud(self):
        
        # Downsample with voxel size
        pcd_down = self.pcd.voxel_down_sample(self.voxel_size)

        # Estimate normal with search radius
        radius_normal = self.voxel_size * 2
        pcd_down.estimate_normals(
            )
        # Compute the FPFH Features
        radius_feature = self.voxel_size * 5
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd_down, pcd_fpfh