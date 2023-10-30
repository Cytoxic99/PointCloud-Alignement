import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import distance
import copy

class vec3d:
    '''This class defines a vector and simple vector operations in 3D space.
       To be used in isVisible function.'''   
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z
    
    def __sub__(self, other):
        return vec3d(self.x-other.x, self.y-other.y, self.z-other.z)
    
    def normalize(self):
        norm = ((self.x)**2 + (self.y)**2 + (self.z)**2)**0.5 
        self.x = self.x / norm
        self.y = self.y / norm
        self.z = self.z / norm
        
    def __str__(self):
        return f"<{self.x}, {self.y}, {self.z}>" 
    
class roomSegmenter:
    def __init__(self, pcd) -> None:
        self.pcd = pcd
        
        
    def isVisible(self, voxels, start, end):
        '''Return 1 in case the end voxel is visible from the start voxel, and 0 otherwise.
        The other voxel is visible if there are no busy voxels in the ray direction starting 
        from the start voxel and terminating in the end voxel.'''
        

        '''voxels: int 3D _numpy.ndarray_ holding the classification information(dense) of the voxels.
            Interior Free Voxels -> 0
            Busy Voxels -> 1
            Exterior Free Voxels -> 2
        start: A tuple for the indices of the start voxel in (X,Y,Z) format.
        end: A tuple for the indices of the end voxel in (X,Y,Z) format.
            
        return: 1 if visible and 0 otherwise. For the base case, if start and end are same, return 1.'''
        
        if start == end:
            return 1
        
        #Initialize variables. 
        #Some variables are representing the same vectors and redundant yet for consistency with [7] they are used.
        start = vec3d(start[0],  start[1], start[2]) #convert to vector
        end = vec3d(end[0], end[1], end[2]) #convert to vector
        m_CW = 1 #sizes of a voxel
        curpos = start #starting point of ray
        raydir = end-start #direction vector of ray
        raydir.normalize()             
        e = vec3d(0,0,0) #starting position of the voxel grid
        stepX, stepY, stepZ = 0, 0, 0 #step sizes: either 1 or -1 depending on the ray direction
        outX, outY, outZ = 0, 0, 0 #limits for ray tracing
        cb = vec3d(0, 0, 0) #cell(voxel) boundary
        X, Y, Z = int(start.x), int(start.y), int(start.z) #indices of the current cell(voxel)
        tmax = vec3d(0,0,0) #the value of t at which the ray crosses the first v/h/. voxel boundary
        tdelta = vec3d(0,0,0) #how far along the ray we must move (in units of t) to travel the width of a voxel.
        
        #Determine step values and cell boundaries.
        if (raydir.x > 0):
            stepX = 1
            outX = end.x + 1
            cb.x = e.x + (X + 1) * m_CW
        else:
            stepX = -1
            outX = end.x - 1
            cb.x = e.x + X * m_CW
        if (raydir.y > 0):
            stepY = 1
            outY = end.y + 1
            cb.y = e.y + (Y + 1) * m_CW
        else:
            stepY = -1
            outY = end.y - 1
            cb.y = e.y + Y * m_CW
        if (raydir.z > 0):
            stepZ = 1
            outZ = end.z + 1
            cb.z = e.z + (Z + 1) * m_CW
        else:
            stepZ = -1
            outZ = end.z - 1
            cb.z = e.z + Z * m_CW
        
        #Determine the tmax and tdelta values.
        if (raydir.x != 0):
            tmax.x = (cb.x - curpos.x) / raydir.x 
            tdelta.x = (m_CW * stepX) / raydir.x
        else:
            tmax.x = 10000
        if (raydir.y != 0):
            tmax.y = (cb.y - curpos.y) / raydir.y 
            tdelta.y = (m_CW * stepY) / raydir.y
        else:
            tmax.y = 10000
        if (raydir.z != 0):
            tmax.z = (cb.z - curpos.z) / raydir.z 
            tdelta.z = (m_CW * stepZ) / raydir.z
        else:
            tmax.z = 10000
        
        #Trace the ray
        while (True):
            if (tmax.x < tmax.y):
                if (tmax.x < tmax.z):
                    X = X + stepX
                    if (X == outX):
                        return 0
                    tmax.x += tdelta.x
                else:
                    Z = Z + stepZ
                    if (Z == outZ):
                        return 0
                    tmax.z += tdelta.z
            else:
                if (tmax.y < tmax.z):
                    Y = Y + stepY
                    if (Y == outY):
                        return 0
                    tmax.y += tdelta.y
                else:
                    Z = Z + stepZ
                    if (Z == outZ):
                        return 0
                    tmax.z += tdelta.z
            if (voxels[X][Y][Z] == 1): #Ray passes through a busy voxel, end voxel is not visible.
                return 0
            if (X == end.x and Y == end.y and Z == end.z): #Visible if ray can be traced until
                return 1
            
    def getAxes(self):
        '''This functions returns a point cloud with color coded points representing 
        the X(Red), Y(Green), Z(Blue) axes and the Origin(Black).'''
        
        axes = o3d.geometry.PointCloud() #get an empty point cloud
        
        # manually define axes points (origin, x1, ..., x4, y1, ..., y4, z1, ... z4)
        np_points = np.zeros((13, 3))
        np_points[0] = [0,0,0]
        np_points[1] = [1,0,0] 
        np_points[2] = [2,0,0]
        np_points[3] = [3,0,0]
        np_points[4] = [4,0,0]
        np_points[5] = [0,1,0]
        np_points[6] = [0,2,0]
        np_points[7] = [0,3,0]
        np_points[8] = [0,4,0]
        np_points[9] = [0,0,1]
        np_points[10] = [0,0,2]
        np_points[11] = [0,0,3]
        np_points[12] = [0,0,4]

        #manually define axes colors (origin->black, X_axis->red, Y_axis->green, Z_axis->blue )
        np_colors = np.zeros((13, 3))
        np_colors[0] = [0,0,0]
        np_colors[1] = [1,0,0]
        np_colors[2] = [1,0,0]
        np_colors[3] = [1,0,0]
        np_colors[4] = [1,0,0]
        np_colors[5] = [0,1,0]
        np_colors[6] = [0,1,0]
        np_colors[7] = [0,1,0]
        np_colors[8] = [0,1,0]
        np_colors[9] = [0,0,1]
        np_colors[10] = [0,0,1]
        np_colors[11] = [0,0,1]
        np_colors[12] = [0,0,1]

        axes.points = o3d.utility.Vector3dVector(np_points)
        axes.colors = o3d.utility.Vector3dVector(np_colors)
        
        return axes

    def load_pcd(self):
    
        print(f"-Point cloud: {self.pcd}\n")
        print(f"-Points in the cloud(X,Y,Z):\n{np.asarray(self.pcd.points)}\n")
        aabb = self.pcd.get_axis_aligned_bounding_box()
        aabb.color = [0,0,0]
        print(f"-{aabb}")
        axes = self.getAxes() #See 2.1 Helper Functions
        o3d.visualization.draw_geometries([self.pcd, aabb, axes])


        #(DIMINISHED) #User defined ratio to determine voxel size
        #(DIMINISHED) v_ratio = 0.009

        #(DIMINISHED) #Determine the voxel size based on the longest edge of the bounding box and a constant ratio. Use 4 decimal point precision.
        #(DIMINISHED) v_size=round(max(self.pcd.get_max_bound()-self.pcd.get_min_bound())*v_ratio, 4)  

        v_size=500
        v_number= (self.pcd.get_max_bound()-self.pcd.get_min_bound())/v_size
        v_number[0], v_number[1], v_number[2] = round(v_number[0])+1, round(v_number[1])+1, round(v_number[2])+1 #+1 due to the calculation of voxel_grid
        v_number = v_number.astype(int)

        #Create a voxel grid with voxels for each subspace of the point cloud with at least one data point.   
        voxel_grid=o3d.geometry.VoxelGrid.create_from_point_cloud(self.pcd,voxel_size=v_size)

        #Transfer busy vs. free information from voxel grid to a 3D 'np.array'. 
        free_voxels = np.zeros((v_number[0], v_number[1], v_number[2]), dtype=np.uint8) 
        for voxel in voxel_grid.get_voxels():
            free_voxels[voxel.grid_index[0]][voxel.grid_index[1]][voxel.grid_index[2]] = 1 #Busy_Voxel->1 (Free_Voxel->0) 

        #Free the isolated busy voxels (new version)
        for plane in free_voxels: #get planes orthogonal to X-axis
            for vertical_stack in plane.T: #get vertical stacks parallel to Y-axis
                for y in range(v_number[1]):
                    if vertical_stack[y] == 1: #if a voxel is busy
                        if y < len(vertical_stack)-4 and vertical_stack[y+1] == 0 and vertical_stack[y+2] == 1 and vertical_stack[y+3] == 0 and vertical_stack[y+4] == 1:
                            vertical_stack[y+2] = 0; #free the isolated voxel

        print(f"Number of busy voxels out of total number of voxels: {len(voxel_grid.get_voxels())}/{v_number[0]*v_number[1]*v_number[2]}")
        o3d.visualization.draw_geometries([voxel_grid, aabb, axes])
        
        pf_2D = np.zeros((v_number[0], v_number[2]))
        pf_3D = np.zeros((v_number[0], v_number[1], v_number[2]))


        #Find the L2(euclidian) norm of every free voxel to its nearest neighbor. Start from the free voxels lying in 
        #the lowest(in terms of Y coords.) plane parallel to XZ-plane. Then continue with the other voxels in 
        #upper planes(in terms of Y coord). By doing so, use the list of busy voxels in
        #the halfspace spanning the positive Y-direction. 
        i = 0 #to trace Y level
        for plane in np.moveaxis(free_voxels, 0, 1): #Each plane is vertical to Y-axis
            #Select busy voxels in the halfspace spanning the positive Y-direction wrt. the Y level of current plane.
            busy_voxels = np.asarray([voxel.grid_index for voxel in voxel_grid.get_voxels() if voxel.grid_index[1]>=i])
            for x in range(v_number[0]):
                for z in range(v_number[2]):
                    if plane[x][z] == 0: #calculate pf for only interior free voxels.
                        pf_3D[x][i][z] = min(distance.cdist(busy_voxels, np.array([[x,i,z]]), 'euclidean'))[0]
            i = i + 1 #increase y to trace Y level of current planes             

            
        #Project pf_3D to XZ-plane (Will result in a dense array with many zero PF valued voxels)
        #Simultaneously, detect the maximum free voxels among each vertical stack with at least one interior free voxelç
        highest_free_voxels = list()    
        for x in range(v_number[0]):
            for z in range(v_number[2]):
                y_index_of_max = np.argmax(pf_3D[x].T[z]) #find index of maximum along a vertical stack
                maximum_pf = pf_3D[x][y_index_of_max][z] #project the maximum to XZ-plane
                pf_2D[x][z] = maximum_pf
                if (maximum_pf > 0): #if there is at least one free voxel at that stack than find the highest free voxel of the stack
                    highest_y = -1
                    for y in range(v_number[1]):
                        if free_voxels[x][y][z] == 0:
                            highest_y = y
                    if highest_y > -1:
                        highest_free_voxels.append((x,y,z))
                        
        #Plot the 2D PF Map (only for the free voxels with PF values)
        cmap = copy.copy(plt.get_cmap('hot'))
        cmap.set_under('white')
        plt.figure(figsize=(15, 15))
        plt.imshow(pf_2D.T, cmap=cmap, vmin=0.1)
        plt.title("2D Anisotropic Potential Field(PF) Map")
        plt.xlabel("X-axis")
        plt.ylabel("Z-axis")
        plt.show()
        
        #Create an empty visib_map list with the shape of pf_2D
        x = v_number[0]
        z = v_number[2]
        visib_map = [0] * x
        for i in range(x):
            visib_map[i] = [0] * z
            
        #Calculate the visiblity vectors for each highest free voxel and record it to visib_map
        for start in highest_free_voxels:
            visib_vec = list()
            for end in highest_free_voxels:
                visib_vec.append(self.isVisible(free_voxels, start, end))
            visib_map[start[0]][start[2]] = visib_vec