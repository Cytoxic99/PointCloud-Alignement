import ifcopenshell as ifc
import ifcopenshell.util.element as elem
import ifcopenshell.util.shape
from ifcopenshell import geom
from icecream import ic
from lib.visualizer import Visualizer
import math




class Segmentation:
    
    def __init__(self, path) -> None:
        self.model = ifc.open(path)
                    
    def wall_pos(self, wall):
        
        if wall.ObjectPlacement.RelativePlacement.RefDirection != None:
            axis = wall.ObjectPlacement.RelativePlacement.RefDirection.DirectionRatios
            
        else:
            axis = [0, 0, 0]

        
        x = []
        y = [] 
        
        coordinates = wall.ObjectPlacement.RelativePlacement.Location.Coordinates
        x = coordinates[0]
        y = coordinates[1]
        
        return [x, y], axis
        
    
    def wall_length(self, wall):
        geometry = wall.Representation.Representations
        for geom in geometry:
            
            for item in geom.Items:
                
                if item.is_a() == "IfcPolyline":
                    Points = item.Points
                    start = Points[0].Coordinates
                    end = Points[1].Coordinates
                    length = math.dist(start, end)
                    ic(start, end, length)
                    
                    
        return length
                
    def calculate_2nd_Point(self, start_point, direction, length):
        
        u = direction[0]
        v = direction[1]
        
        x2 = start_point[0] + length*u
        y2 = start_point[1] + length*v
        
        return [x2, y2]
    
    def wall_info(self, wall):
        matrix = ifcopenshell.util.shape.get_profiles(wall)
        ic(matrix)
        
        
                    
    def calc_wall(self):
        vis = Visualizer()
        
        walls = self.model.by_type("IfcWall")
        
        starting_points = []
        ending_points = []
        for wall in walls:
            self.wall_info(wall)
            '''
            start_point, direction = self.wall_pos(wall)
            starting_points.append(start_point)
            length = self.wall_length(wall)
            
            ending_points.append(self.calculate_2nd_Point(start_point, direction, length))
        
        vis.plot_lines(starting_points, ending_points)
           '''
