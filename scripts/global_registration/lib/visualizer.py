import open3d as o3d
import copy

import matplotlib.pyplot as plt
from icecream import ic

class Visualizer:
    def __init__(self) -> None:
        self.i = 0
    
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
        x = matrix[:, 0].astype(float)
        y = matrix[:, 1].astype(float)
        
        try:
            plt.scatter(x, y, label='Point Cloud')
        except Exception as e:
            # Convert x and y to lists
            ic(e)
            return
        
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
        
        plt.show()
        
    def draw_pointcloud_and_rectangles(self, matrix, rectangles):
        x = matrix[:, 0]
        y = matrix[:, 1]
        plt.scatter(x, y, label='Point Cloud')

        for i, rectangle_points in enumerate(rectangles):
            try:
                rectangle_x = [point[0] for point in rectangle_points]
                rectangle_y = [point[1] for point in rectangle_points]
            except:
                rectangle_x = rectangle_points[:, 0]
                rectangle_y = rectangle_points[:, 1]

            # Connect the rectangle points to form a closed shape
            rectangle_x.append(rectangle_x[0])  # Closing the shape
            rectangle_y.append(rectangle_y[0])

            # Choose a unique color for each rectangle
            color = plt.cm.viridis(i / len(rectangles))

            # Create lines for the rectangle with a unique color
            plt.plot(rectangle_x, rectangle_y, color=color, label=f'Rectangle {i}')

        # Set labels and title
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('2D Point Cloud Visualization')

        # Display the legend
        plt.legend()

        plt.show()
        

        