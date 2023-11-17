import open3d as o3d
import copy
import matplotlib.pyplot as plt

class Visualizer:
    def __init__(self) -> None:
        pass
    
    def draw_registration_result(self, floorModel, roomScan, transformation):
        floorModel_temp = copy.deepcopy(floorModel)
        roomScan_temp = copy.deepcopy(roomScan)
        floorModel_temp.paint_uniform_color([1, 0.706, 0])
        roomScan_temp.paint_uniform_color([0, 0.651, 0.929])
        floorModel_temp.transform(transformation)
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
        
    
    def draw_pointcloud_and_lines(self, matrix, points):
        x = matrix[:, 0]
        y = matrix[:, 1]
        plt.scatter(x, y, label='Point Cloud')
        
        try:
            rectangle_x = [point[0] for point in points]
            rectangle_y = [point[1] for point in points]

        except:
            rectangle_x = points[:, 0]
            rectangle_y = points[:, 1]
        
        # Connect the rectangle points to form a closed shape
        rectangle_x.append(rectangle_x[0])  # Closing the shape
        rectangle_y.append(rectangle_y[0])
        
        # Create lines for the rectangle
        plt.plot(rectangle_x, rectangle_y, color='red', label='Rectangle')
        
        # Set labels and title
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('2D Point Cloud Visualization')
        
        # Display the legend
        plt.legend()
        
        plt.show(block=True)
        plt.pause(0.1)
        # Close the plot
        plt.close()

        