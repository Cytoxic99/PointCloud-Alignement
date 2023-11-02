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
        floorModel = o3d.io.read_point_cloud(self.model_pcd)
        roomScan = o3d.io.read_point_cloud(self.scan_pcd)
        trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        floorModel.transform(trans_init)
        
        # Preprocess the point clouds
        processor_floorModel = Preprocessor(floorModel, voxel_size)
        processor_roomScan = Preprocessor(roomScan, voxel_size)
        floorModel_down = processor_floorModel.preprocess_point_cloud()
        roomScan_down = processor_roomScan.preprocess_point_cloud()
        
        return floorModel, roomScan, floorModel_down, roomScan_down