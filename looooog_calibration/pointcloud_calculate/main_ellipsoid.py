#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import os
import copy
import cv2 as cv
import objectPoint
import pointEllipsoid

Abs_Path=os.path.dirname(os.path.abspath(__file__))
cloud0 = objectPoint.objectToPoint(Abs_Path,0,1)
transformed_cloud = objectPoint.transform(cloud0)
ellipsoid = pointEllipsoid.EllipsoidTool()
center, radii, rotation = ellipsoid.getMinVolEllipse(transformed_cloud)
print(center,radii,rotation)
ellipsoid_point = ellipsoid.getEllipsoidPoint(center,radii,rotation)
pointEllipsoid.talker(ellipsoid_point)
# print(radii)
# [-799.43973271,  -21.71781789,   91.97020756]), array([ 31.20462948,  50.19825781, 161.52593938]