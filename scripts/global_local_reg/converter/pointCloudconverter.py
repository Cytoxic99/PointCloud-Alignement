
class pointCloudConverter:

    def __init__(self, inputFile, density_factor=2) -> None:
        self.inputFile = inputFile
        self.density_factor = density_factor

    # Function to extract vertices from an OBJ file and save them as a .xyz point cloud
    def obj_to_xyz_point_cloud(self, outputFile):
        # Initialize an empty list to store the vertices
        vertices = []
        faces = []

        # Open and read the OBJ file
        with open(self.inputFile, 'r') as file:
            lines = file.readlines()

        # Extract vertex coordinates from the OBJ file
        for line in lines:
            if line.startswith('v '):
                _, x, y, z = line.split()
                vertices.append([float(x), float(y), float(z)])
            
            elif line.startswith('f '):
                _, v1, v2, v3 = line.split()
                faces.append([int(v1.split('/')[0]) - 1, int(v2.split('/')[0]) - 1, int(v3.split('/')[0]) - 1])

        # Interpolate additional points on the edges based on the density factor
        interpolated_vertices = []
        for face in faces:
            for i in range(3):
                v1 = vertices[face[i]]
                v2 = vertices[face[(i + 1) % 3]]
                for j in range(1, self.density_factor):
                    t = j / self.density_factor
                    interpolated_point = [v1[k] + t * (v2[k] - v1[k]) for k in range(3)]
                    interpolated_vertices.append(interpolated_point)
        # Combine original vertices and interpolated vertices
        
        all_vertices = vertices + interpolated_vertices

        # Save the vertices as a .xyz point cloud
        with open(outputFile, 'w') as file:
            for point in all_vertices:
                file.write(f'{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n')



if __name__ == "__main__":
    Converter = pointCloudConverter('data\Scaled_DFloor.obj', 10)
    Converter.obj_to_xyz_point_cloud('data\DFloor.xyz')