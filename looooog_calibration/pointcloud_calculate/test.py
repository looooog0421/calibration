#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import open3d as o3d
import matplotlib.pyplot as plt # plt 用于显示图片
import numpy as np
import os
 
 
#一共转换36张图
for i in range(1):
    abs_path=os.path.dirname(os.path.abspath(__file__))
    color_raw = o3d.io.read_image(os.path.join(abs_path, "images", "color_1.png"))
    depth_raw = o3d.io.read_image(os.path.join(abs_path, "images", "depth_1.png"))
    # 这一步因为我的深度图的值都在10000左右，但depth_scale默认值是1000，我就把depth_trunc从默认的3改成30，深度就显示出来，原理不是很懂
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw, depth_scale=1000.0, depth_trunc=30, convert_rgb_to_intensity=False)
    # plt.subplot(1, 2, 1)
    # plt.title('read_depth')
    # plt.imshow(rgbd_image.color)
    # plt.subplot(1, 2, 2)
    # plt.title('depth image')
    # plt.imshow(rgbd_image.depth)
    # plt.show()
    # 若要查看自己的深度图值是多少，使用下面的np函数显示
    # print(np.asarray(rgbd_image.depth))
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    print("第%d张点云数据:"%i, pcd)
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    o3d.visualization.draw_geometries([pcd])
    # o3d.io.write_point_cloud("./pcldata/living_room/pcd/%d.pcd" %i, pcd)




    ###test my github