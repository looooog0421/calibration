#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

# import numpy as np
# import rospy
# from sensor_msgs.msg import PointCloud2
# from sensor_msgs.msg import PointField
import os
# import copy
# import cv2 as cv
import objectPoint
import open3d as o3d


Abs_Path=os.path.dirname(os.path.abspath(__file__))
cloud = objectPoint.objectToPoint(Abs_Path,0,1)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(cloud)
o3d.io.write_point_cloud("objectPoint.pcd",pcd)
transformed_cloud = objectPoint.transform(cloud)
print("第一第二第三",transformed_cloud)
print(cloud)
objectPoint.talker(transformed_cloud)
