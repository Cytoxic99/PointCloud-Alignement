from preparer import Preparer
from visualizer import Visualizer
from registrator import Registrate
from clusterer import Clusterer
import open3d as o3d
import numpy as np
from icecream import ic
from room_segmentation import RoomFinder


if __name__ == "__main__":
    visualizer = Visualizer()
    
    model_path = "data\DFloor.xyz"
    scan_path = "data\Scaled_SpatialMapping.xyz"
    voxel_size = 100
    
    source, target, source_down, target_down = Preparer(model_path, scan_path).prepare_dataset(voxel_size)
    
    #Extract Table and other features from pcd
    #clusterer = Clusterer(target_down)
    #pcd, _ = clusterer.clustering()
    #visualizer.draw_pointcloud(pcd)
    
    registrator = Registrate(source_down, target_down, voxel_size)
   
    rotation_matrix = registrator.registrate_rotation()
    
    source.transform(rotation_matrix)
    source_down.transform(rotation_matrix)
    
    room_finder = RoomFinder(source_down, target_down).getPosition()
    