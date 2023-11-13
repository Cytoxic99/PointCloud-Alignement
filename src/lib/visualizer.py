import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Visualizer:
    
    def __init__(self) -> None:
        pass
    
    def draw_3d(self, x, y, z):
        # Create a 3D scatter plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(x, y, z, c='b', marker='o')

        # Set labels for the axes
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        
        # Show the plot
        plt.show()
        
    def plot_lines(self, starting_points, ending_points):
        for start, end in zip(starting_points, ending_points):
            x1, y1 = start
            x2, y2 = end
            plt.plot([x1, x2], [y1, y2], marker='o')

        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('Lines Visualization')
        plt.grid(True)
        plt.show()