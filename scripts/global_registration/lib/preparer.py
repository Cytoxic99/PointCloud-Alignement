import open3d as o3d
import numpy as np
from lib.visualizer import Visualizer
from lib.preprocesser import Preprocessor
from icecream import ic

class Preparer:
    
    def __init__(self, model_path, scan_path) -> None:
        self.model_pcd = model_path
        self.scan_pcd = scan_path
    
    def prepare_dataset(self, voxel_size):
        #load models and set initial pose
        floorModel = o3d.io.read_point_cloud(self.model_pcd)
        roomScan = o3d.io.read_point_cloud(self.scan_pcd)
        
        # Preprocess the point clouds
        processor_floorModel = Preprocessor(floorModel, voxel_size)
        processor_roomScan = Preprocessor(roomScan, voxel_size)
    
        floorModel_down = processor_floorModel.preprocess_point_cloud()
        roomScan_down = processor_roomScan.preprocess_point_cloud()
        
        return floorModel, roomScan, floorModel_down, roomScan_down