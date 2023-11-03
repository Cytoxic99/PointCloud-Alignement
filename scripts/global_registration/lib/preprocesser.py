import open3d as o3d
import numpy as np
from lib.visualizer import Visualizer

class Preprocessor:
    
    def __init__(self, point_cloud, voxel_size) -> None:
        self.pcd =point_cloud
        self.voxel_size = voxel_size
    
    def preprocess_point_cloud(self):
        
        # Downsample with voxel size
        pcd_down = self.pcd.voxel_down_sample(self.voxel_size)

        return pcd_down
    
    def convert2D(self):
        
        projected_points = np.asarray(self.pcd.points)
        projected_points[:, 1] = 0
        Visualizer().draw_pointcloud(projected_points)
    
    def plot_down(self):
        self.pcd = self.preprocess_point_cloud()
        self.convert2D()