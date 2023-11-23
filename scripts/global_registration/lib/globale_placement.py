from shapely.geometry import Polygon
import numpy as np

class Global:
    
    def __init__(self, rectangles, points_room) -> None:
        self.rectangles = rectangles
        self.points_room = points_room
        
        
    def best_fit(self):
        best_area = np.inf
        best_rectangle = None
        area_room = Polygon(self.points_room).area
        
        for rectangle in self.rectangles:
            current_area = Polygon(rectangle).area
            
            if abs(current_area - area_room) < best_area:
                best_area = abs(current_area - area_room)
                best_rectangle = rectangle
                
        return best_rectangle
    
    def get_transformation(self, rectangle):
        centroid_rectangle = Polygon(rectangle).centroid
        centroid_room = Polygon(self.points_room).centroid
        
        # Calculate the translation vector
        translation_vector = np.array([centroid_room.x - centroid_rectangle.x, centroid_room.y - centroid_rectangle.y])

        # Create the affine transformation matrix
        T = np.array([
            [1, 0, 0, -translation_vector[0]],
            [0, 1, 0, -translation_vector[1]],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        return T
    
    
    def start(self):
        rectangle = self.best_fit()
        T = self.get_transformation(rectangle)
        
        return T, rectangle
        
        
                
            
            