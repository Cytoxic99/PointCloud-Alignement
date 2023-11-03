import open3d as o3d
import copy

class Visualizer:
    def __init__(self) -> None:
        pass
    
    def draw_registration_result(self, floorModel, roomScan, transformation):
        floorModel_temp = copy.deepcopy(floorModel)
        roomScan_temp = copy.deepcopy(roomScan)
        floorModel_temp.paint_uniform_color([1, 0.706, 0])
        roomScan_temp.paint_uniform_color([0, 0.651, 0.929])
        roomScan_temp.transform(transformation)
        o3d.visualization.draw_geometries([floorModel_temp, roomScan_temp],
                                        zoom=0.4559,
                                        front=[0.6452, -0.3036, -0.7011],
                                        lookat=[1.9892, 2.0208, 1.8945],
                                        up=[-0.2779, -0.9482, 0.1556])
        
    def draw_pointcloud(self, floorModel):
        o3d.visualization.draw_geometries([floorModel],
                                        zoom=0.4559,
                                        front=[0.6452, -0.3036, -0.7011],
                                        lookat=[1.9892, 2.0208, 1.8945],
                                        up=[-0.2779, -0.9482, 0.1556])
        
    
    def draw_multiple_pointclouds(self, floorModel):
        
        o3d.visualization.draw_geometries(floorModel)