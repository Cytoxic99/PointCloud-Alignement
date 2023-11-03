from lib.preparer import Preparer
from lib.visualizer import Visualizer
from lib.registrator import Registrate
from lib.clusterer import Clusterer
import open3d as o3d
import numpy as np
from icecream import ic
from lib.room_segmentation import RoomFinder


if __name__ == "__main__":
    visualizer = Visualizer()
    
    model_path = "data\DFloor.xyz"
    scan_path = "data\Scaled_SpatialMapping.xyz"
    voxel_size = 100
    
    floorModel, roomScan, floorModel_down, roomScan_down = Preparer(model_path, scan_path).prepare_dataset(voxel_size)
    
    #Extract Table and other features from pcd
    #clusterer = Clusterer(roomScan_down)
    #pcd, _ = clusterer.clustering()
    #visualizer.draw_pointcloud(pcd)
    
    '''
    OPTIMAL PARAMAETERS
    x_translation = -8500
    z_translation = -3000
    
    rotation_angle = np.pi*(21/36)


    registrator = Registrate(floorModel_down, roomScan_down, voxel_size)
   
    rotation_matrix = registrator.registrate_rotation()
    
    floorModel.transform(rotation_matrix)
    floorModel_down.transform(rotation_matrix)
    
    # Create the transformation matrix for the random translation
    T = np.identity(4)
    T[0, 3] = x_translation
    T[2, 3] = z_translation
    T[:, 0] = np.array([np.cos(rotation_angle), 0, np.sin(rotation_angle), 0])
    T[:, 2] = np.array([-np.sin(rotation_angle), 0, np.cos(rotation_angle), 0])
    
    visualizer.draw_registration_result(floorModel_down, roomScan_down, T)
    
    '''
    
    registrator = Registrate(floorModel_down, roomScan_down, voxel_size)
   
    rotation_matrix = registrator.registrate_rotation()
    
    floorModel.transform(rotation_matrix)
    floorModel_down.transform(rotation_matrix)
    
    room_finder = RoomFinder(floorModel_down, roomScan_down).getPosition()
    
    