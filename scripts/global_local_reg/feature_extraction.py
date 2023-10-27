import open3d as o3d
import numpy as np
from icecream import ic

# Load the room point cloud and the digital table model
room_cloud = o3d.io.read_point_cloud("data\Scaled_SpatialMapping.xyz")
table_model = o3d.io.read_point_cloud("data\model_tisch.xyz")

# Downsample the point clouds (optional)
room_cloud = room_cloud.voxel_down_sample(voxel_size=10)
table_model = table_model.voxel_down_sample(voxel_size=10)

# Estimate normals for both point clouds
room_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=100, max_nn=30))
table_model.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=100, max_nn=30))

# Compute FPFH features for both point clouds
radius_feature = 0.1
ic("Reached feature extraction")
fpfh_room = o3d.pipelines.registration.compute_fpfh_feature(room_cloud, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
fpfh_table = o3d.pipelines.registration.compute_fpfh_feature(table_model, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

# Perform registration to estimate the transformation matrix
result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
    room_cloud,  # Source point cloud
    table_model,  # Target point cloud
    fpfh_room,  # Source feature (FPFH for room_cloud)
    fpfh_table,  # Target feature (FPFH for table_model)
    mutual_filter=True,  # Enable mutual correspondence filtering
    max_correspondence_distance=10,  # Maximum correspondence distance for RANSAC
    ransac_n=4,  # Number of RANSAC iterations
    criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(400000, 0.999)  # RANSAC criteria
)
ic("Ended the algo")
# Retrieve the estimated transformation matrix
transformation_matrix = result.transformation

# Apply the transformation to the digital table to estimate its position
table_model.transform(transformation_matrix)

# Visualize the aligned table and room
o3d.visualization.draw_geometries([room_cloud, table_model])

# Save the aligned point cloud
o3d.io.write_point_cloud("aligned_room_with_table.pcd", room_cloud + table_model)

