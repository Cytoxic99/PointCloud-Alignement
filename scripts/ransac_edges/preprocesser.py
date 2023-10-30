import open3d as o3d

class Preprocessor:
    
    def __init__(self, point_cloud, voxel_size) -> None:
        self.pcd =point_cloud
        self.voxel_size = voxel_size
    
    def preprocess_point_cloud(self):
        
        # Downsample with voxel size
        pcd_down = self.pcd.voxel_down_sample(self.voxel_size)

        return pcd_down