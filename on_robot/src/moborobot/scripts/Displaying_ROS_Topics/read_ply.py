import open3d as o3d
import numpy as np

ply_path="/home/semooww/catkin_ws/src/staj/scripts/saved_data/pcd/2/lidar-2022_07_21-11_36_02_705.pcd"
pcd=o3d.io.read_point_cloud(ply_path)

o3d.visualization.draw_geometries([pcd])

as_numpy_points=np.asarray(pcd.points)
as_numpy_colors=np.asarray(pcd.colors)


print(as_numpy_points)
print(as_numpy_points.shape)



print(as_numpy_colors)
print(as_numpy_colors.shape)