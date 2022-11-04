#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os


abs_path=os.path.dirname(os.path.abspath(__file__))
color_raw = o3d.io.read_image(os.path.join(abs_path, "images", "color_1.png"))
depth_raw = o3d.io.read_image(os.path.join(abs_path, "images", "depth_1.png"))
# 这一步因为我的深度图的值都在10000左右，但depth_scale默认值是1000，我就把depth_trunc从默认的3改成30，深度就显示出来，原理不是很懂
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw, depth_scale=1000.0, depth_trunc=30, convert_rgb_to_intensity=False)
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,o3d.camera.PinholeCameraIntrinsic(
        o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))


# with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))
# 求点云的聚类数量
max_label = np.max(labels) + 1
print(max(labels))
# print("point cloud has {max_label + 1} clusters")
# 可视化
colors = np.random.randint(255, size=(max_label, 3))/255.
print(colors)
colors = colors[labels]
print(colors.shape) 
print(labels.shape)
pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
# o3d.visualization.draw_geometries([pcd],
#                                   zoom=0.455,
#                                   front=[-0.4999, -0.1659, -0.8499],
#                                   lookat=[2.1813, 2.0619, 2.0999],
#                                   up=[0.1204, -0.9852, 0.1215])


o3d.visualization.draw_geometries([pcd])
