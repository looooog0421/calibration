#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import open3d as o3d
import numpy as np
from sklearn.cluster import KMeans
import os
 
 
if __name__ == '__main__':
    abs_path=os.path.dirname(os.path.abspath(__file__))
    color_raw = o3d.io.read_image(os.path.join(abs_path, "images", "color_1.png"))
    depth_raw = o3d.io.read_image(os.path.join(abs_path, "images", "depth_1.png"))
    # 这一步因为我的深度图的值都在10000左右，但depth_scale默认值是1000，我就把depth_trunc从默认的3改成30，深度就显示出来，原理不是很懂
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw, depth_scale=1000.0, depth_trunc=30, convert_rgb_to_intensity=False)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))


    # pcd = pcd.uniform_down_sample(50)#每50个点采样一次
    pcd.paint_uniform_color([0.5, 0.5, 0.5])#指定显示为灰色
    print(pcd)
    
    points = np.array(pcd.points)
    result = KMeans(n_clusters=8).fit(points)
    #各个类别中心
    center = result.cluster_centers_
    # labels返回聚类成功的类别，从0开始，每个数据表示一个类别
    labels = result.labels_
     
    #最大值相当于共有多少个类别
    max_label = np.max(labels) + 1 #从0开始计算标签
    print(max(labels))
    #生成k个类别的颜色，k表示聚类成功的类别
    colors = np.random.randint(255, size=(max_label, 3))/255.
    colors = colors[labels]
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
 
    # 点云显示
    o3d.visualization.draw_geometries([pcd]), #点云列表