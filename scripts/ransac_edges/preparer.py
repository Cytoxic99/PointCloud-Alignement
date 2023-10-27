import open3d as o3d
import numpy as np
from visualizer import Visualizer
from preprocesser import Preprocessor

class Preparer:
    
    def __init__(self, model_path, scan_path) -> None:
        self.model_pcd = model_path
        self.scan_pcd = scan_path
    
    def prepare_dataset(self, voxel_size):

        #load models and set initial pose
        source = o3d.io.read_point_cloud(self.model_pcd)
        target = o3d.io.read_point_cloud(self.scan_pcd)
        trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        source.transform(trans_init)
        
        # Preprocess the point clouds
        processor_source = Preprocessor(source, voxel_size)
        processor_target = Preprocessor(target, voxel_size)
        source_down, source_fpfh = processor_source.preprocess_point_cloud()
        target_down, target_fpfh = processor_target.preprocess_point_cloud()
        
        return source, target, source_down, target_down, source_fpfh, target_fpfh