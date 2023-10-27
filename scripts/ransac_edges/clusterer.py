import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

class Clusterer:
    
    def __init__(self, pcd) -> None:
        self.pcd = pcd
    
    
    def clustering(self):
        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(
                self.pcd.cluster_dbscan(eps=1000, min_points=10, print_progress=True))

        max_label = labels.max()
        print(f"point cloud has {max_label + 1} clusters")
        colors = plt.get_cmap("jet")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        self.pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        
        clusters = []
        for label in range(max_label + 1):
            # Extract the points belonging to the current cluster
            cluster_indices = np.where(labels == label)[0] 
            cluster_points = self.pcd.select_by_index(cluster_indices)
            
            # Append the cluster to the list of clusters
            clusters.append(cluster_points)

        return self.pcd, clusters
        
        