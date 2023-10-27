import open3d as o3d
import numpy as np

class MeshScaler:
    
    def __init__(self, spatialMesh, outputFile) -> None:
        self.mesh = spatialMesh
        self.output = outputFile
    
    def scale(self, scaling_factor=1000):
        
        # Load the .obj file
        mesh = o3d.io.read_triangle_mesh(self.mesh)

        # Extract vertices and faces
        vertices = mesh.vertices
        faces = mesh.triangles

        scaled_vertices = np.asarray(vertices) * scaling_factor

        scaled_mesh = o3d.geometry.TriangleMesh()
        scaled_mesh.vertices = o3d.utility.Vector3dVector(scaled_vertices)
        scaled_mesh.triangles = o3d.utility.Vector3iVector(faces)

        o3d.io.write_triangle_mesh(self.output, scaled_mesh)
        
if __name__ == "__main__":
    scaler = MeshScaler("data\DFloor.obj", "data\Scaled_DFloor.obj").scale(scaling_factor=1000)