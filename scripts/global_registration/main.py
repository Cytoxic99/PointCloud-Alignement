from lib.preparer import Preparer
from lib.visualizer import Visualizer
from lib.registrator import Registrate
import numpy as np
from icecream import ic
from lib.segmentor import Segmentor
from lib.globale_placement import Global
from lib.local_placement import Local
import os


if __name__ == "__main__":
    ic()
    visualizer = Visualizer()
    
    model_path = "data\DFloor.xyz"
    scan_path = "data\Scaled_SpatialMapping.xyz"
    voxel_size = 100
    
    
    floorModel, roomScan, floorModel_down, roomScan_down = Preparer(model_path, scan_path).prepare_dataset(voxel_size)
    
    
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
    T_reg, floorModel_2d, roomScan_2d, points_floor, points_room = registrator.registrate()
    
    if not os.path.exists('rectangles.npy'):
        rectangles = Segmentor(floorModel_2d, points_floor).start()
        # Save the array to a file
        np.save('rectangles.npy', rectangles)
    else:
        rectangles = np.load('rectangles.npy')
    
    angle = Segmentor(floorModel_2d, points_floor).get_angle()
        
    T_place, rectangle = Global(rectangles, points_room).start()
    
    
    Local(floorModel_down, roomScan_down, floorModel_2d, roomScan_2d, rectangle, points_room, angle).start()
    
    
    
    #Ransac(floorModel_2d, roomScan_2d, points_floor, points_room).start()
    #transform = RoomFinder(floorModel_down, roomScan_down).getPosition()
    
    ic()
    
    
