import open3d as o3d
import numpy as np
from ransac import RANSACMatching

print("----- START MERGE -----")

root_model_path = "data\Scaled_SpatialMapping.obj"
moving_model_path = "data\model_tisch.obj"

# Load the root model (fixed)
root_model = o3d.io.read_triangle_mesh(root_model_path)

# Load the model to be moved
moving_model = o3d.io.read_triangle_mesh(moving_model_path)

# Define the transformation matrices for translation and rotation
transformation = RANSACMatching(moving_model_path, root_model_path).match()

# Apply the transformation to the moving model
moving_model.transform(transformation)

# Merge the two models
merged_model_p2p = root_model + moving_model


# Save the merged model to a new .obj file
o3d.io.write_triangle_mesh("data\merged_model_p2p.obj", merged_model_p2p)
print("----- MODEL SAVED -----")

