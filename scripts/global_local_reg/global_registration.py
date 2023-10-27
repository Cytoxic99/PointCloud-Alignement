import open3d as o3d
import numpy as np
import copy
import os
import sys
import copy
from icecream import ic
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.0, 1])
    source_temp.transform(transformation)
    #o3d.visualization.draw_geometries([source_temp, target_temp], zoom=0.4559, front=[0.6457, -0.3036, -0.7011], lookat= [1.9892, 2.0208, 1.8945], up=[-0.278, -0.95, 0.1556])
    
def detect_walls_parallel(point_cloud, iterations=1000, remove_threshold=300, num_processes=8):
    
    def find_best_line_chunk(iterations_chunk):
        # Inside this function, we will perform a chunk of iterations
        
        best_line_amount = 0  # Initialize to 0
        best_current_points = []
        distance_threshold = 0.1
        
        for _ in range(iterations_chunk):
            # Generate two random indices
            random_index1, random_index2 = np.random.choice(num_points, 2, replace=False)

            # Get the corresponding random points
            point1 = projected_points[random_index1]
            point2 = projected_points[random_index2]

            # Initialize a counter for points within the threshold
            points_within_threshold = 0
            current_points = []
            # Iterate through your point cloud to check each point
            for k in range(num_points - 1):
                x = projected_points[k][0]
                y = projected_points[k][2]
                
                # Calculate the perpendicular distance from the point to the line
                distance = np.abs((point2[0] - point1[0]) * (point1[2] - y) - (point1[0] - x) * (point2[2] - point1[2])) / np.sqrt((point2[0] - point1[0]) ** 2 + (point2[2] - point1[2]) ** 2)

                # Check if the point is within the threshold distance
                if distance <= distance_threshold:
                    points_within_threshold += 1
                if distance <= remove_threshold:
                    current_points.append(k)
                if current_points == 100:
                    break
            
            # Check if this line has more inliers than the best line found so far
            if points_within_threshold > best_line_amount:
                best_line_amount = points_within_threshold
                best_point1 = point1
                best_point2 = point2
                best_current_points = current_points
        
        return best_point1, best_point2, best_current_points

    # Create a copy of the point cloud
    point_cloud_copy = copy.copy(point_cloud)
    num_points = point_cloud_copy.points.shape[0]
    projected_points = np.asarray(point_cloud_copy.points)
    projected_points[:, 1] = 0  # Set Z-coordinates to zero

    with Pool(num_processes) as pool:
        while True:
            # Split the iterations into chunks
            chunk_size = iterations // num_processes
            chunks = [chunk_size] * num_processes
            last_chunk_size = iterations % num_processes
            if last_chunk_size:
                chunks.append(last_chunk_size)

            best_line_amount = 0
            best_current_points = []

            # Perform iterations in parallel
            results = pool.map(find_best_line_chunk, chunks)

            for result in results:
                if result[2] > best_line_amount:
                    best_line_amount, best_point1, best_point2, best_current_points = result

            if len(best_current_points) < 300:
                print("All walls found")
                break

            # Visualize the best line
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector([best_point1, best_point2])
            line_set.lines = o3d.utility.Vector2iVector([[0, 1]])
       
            filtered3d_point_cloud = point_cloud.select_by_index(best_current_points, invert=True)
            filtered2d_point_cloud = point_cloud_copy.select_by_index(best_current_points, invert=True)
        
            point_cloud_copy = filtered2d_point_cloud
            point_cloud = filtered3d_point_cloud

    o3d.visualization.draw_geometries([point_cloud])
    
    return point_cloud

    
def segmentation(pcd, max_iterations):
    num = 2
    for i in range(num):
        best_inliers = None
        for _ in range(max_iterations):
            # Plane model segmentation with RANSAC
            plane_model, inliers = pcd.segment_plane(100, ransac_n=3,num_iterations=10000)
            inlier_cloud = pcd.select_by_index(inliers, invert=True)
            
            if best_inliers == None:
                best_inliers = inliers
            
            # Check if this model has more inliers than the previous best
            if len(inliers) > len(best_inliers):
                best_inliers = inliers
            
        inlier_cloud = pcd.select_by_index(best_inliers, invert=True)
        pcd = inlier_cloud
    #o3d.visualization.draw_geometries([inlier_cloud])
    return inlier_cloud

def preprocess_point_cloud(pcd, roomScan, voxel_size, Preprocessing=True):
    #down sample with the voxelsize
    if Preprocessing:
        pcd = pcd.voxel_down_sample(voxel_size)
    if roomScan:
        pcd = segmentation(pcd, 10)

    #Estimate normal with search radius
    radius_normal = voxel_size*2
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    #compute FPFH feature with seach radius
    radius_feature = voxel_size*5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    
    # Create a color map for the FPFH features
    fpfh_color = o3d.geometry.PointCloud()
    fpfh_color.points = pcd.points
    fpfh_color.colors = o3d.utility.Vector3dVector(pcd_fpfh.data[:, :3])  # Use the first 3 columns of FPFH data

    # Visualize the FPFH features
    o3d.visualization.draw_geometries([fpfh_color])

    return pcd, pcd_fpfh


def prepare_dataset(voxel_size):
    #load 2 point clouds and disturb initial pose
    source = o3d.io.read_point_cloud("data\model_tisch.xyz")
    target = o3d.io.read_point_cloud("data\Scaled_SpatialMapping.xyz")
    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    
    source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))
    
    source_down, _ = preprocess_point_cloud(source, False, voxel_size)
    target_down, _ = preprocess_point_cloud(target, True, voxel_size)
    
    return source, target, source_down, target_down

#We use RANSAC for the global localization
def execute_global_registration(source_down, target_down, voxel_size):
    distance_threshold = voxel_size
    #RANSAC registration on downsampled point clouds. Since the downsampling voxel size is voxel_size we use a liberal distance threshld distance_thresold
    
    _, source_fpfh = preprocess_point_cloud(source_down, False, voxel_size, False)
    _, target_fpfh = preprocess_point_cloud(target_down, False, voxel_size, False)

    print("Enter RANSAC")
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(source_down, target_down, source_fpfh, target_fpfh, True, distance_threshold, 
                                                                                     o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
                                                                                     [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                                                                                      o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
                                                                                     o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    print("Finished RANSAC")
    return result



if __name__ == "__main__":
    
    root_model_path = "data\Scaled_SpatialMapping.obj"
    moving_model_path = "data\model_tisch.obj"

    # Load the root model (fixed)
    root_model = o3d.io.read_triangle_mesh(root_model_path)

    # Load the model to be moved
    moving_model = o3d.io.read_triangle_mesh(moving_model_path)

    voxel_size = 100
    source, target, source_down, target_down = prepare_dataset(voxel_size)
    target_down = detect_walls_parallel(target_down)
    result_ransac = execute_global_registration(source_down, target_down, voxel_size)
    
    draw_registration_result(source_down, target_down, result_ransac.transformation)
    
    # Apply the transformation to the moving model
    moving_model.transform(result_ransac.transformation)

    # Merge the two models
    merged_model = root_model + moving_model


    # Save the merged model to a new .obj file
    o3d.io.write_triangle_mesh("data\merged_model_global.obj", merged_model)
    print("----- MODEL SAVED -----")
        
    