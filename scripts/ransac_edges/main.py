from preparer import Preparer
from visualizer import Visualizer
from registrator import Registrate
from clusterer import Clusterer
import open3d as o3d
import numpy as np
from icecream import ic


if __name__ == "__main__":
    visualizer = Visualizer()
    
    model_path = "data\DFloor.xyz"
    scan_path = "data\Scaled_SpatialMapping.xyz"
    voxel_size = 100
    
    source, target, source_down, target_down, source_fpfh, target_fpfh = Preparer(model_path, scan_path).prepare_dataset(voxel_size)
    
    registrator = Registrate(source_down, target_down, source_fpfh, target_fpfh, voxel_size)

    #registrator.execute_global_registration()
    #visualizer.draw_registration_result(source_down, target, registrator.globalTransformation)
   
    rotation_matrix_4x4 = np.eye(4)
    rotation_matrix = registrator.registrate_rotation()
    rotation_matrix_4x4[:3, :3] = rotation_matrix
    rotation_matrix_4x4[3, 3] = 1
    
    visualizer.draw_registration_result(source, target, rotation_matrix_4x4)
    
    