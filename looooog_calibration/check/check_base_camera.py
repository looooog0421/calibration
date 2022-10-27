#!/usr/bin/python
# -*- coding: utf-8 -*-
import threading
from copy import deepcopy
import yaml
import rospy
import roslib
import tf
import numpy as np
from geometry_msgs.msg import Pose
import os
import cv2

def pose2matrix(pose): #根据姿态生成机械臂基座到末端的变换矩阵
    Position = pose[0:3].reshape(3,1)
    qx = pose[3]
    qy = pose[4]
    qz = pose[5]
    qw = pose[6]

    Rot = np.array([[2 * qw ** 2 + 2 * qx ** 2 - 1, 2 * qx * qy - 2 * qw * qz, 2 * qx * qz + 2 * qw * qy],
                    [2 * qx * qy + 2 * qw * qz, 2 * qw ** 2 + 2 * qy ** 2 - 1, 2 * qy * qz - 2 * qw * qx],
                    [2 * qx * qz - 2 * qw * qy, 2 * qy * qz + 2 * qw * qx, 2 * qw ** 2 + 2 * qz ** 2 - 1]])

    TransformationMatrix = np.vstack((np.hstack((Rot, Position)),
                                        np.array([0,0,0,1])))
    return TransformationMatrix
#读取所需的文件
filefolder = os.path.dirname(os.path.dirname(__file__)) #上级文件夹
poseeef_base = np.load(os.path.join(filefolder, 'calibration_files', 'Poseeef_base.npy')) #机械臂末端在机械臂坐标系下的位置和姿态
Ttip_eef = np.load(os.path.join(filefolder, 'calibration_files', 'transform_Ttip_eef.npy')) #机械臂末端到探针的变换矩阵
Tpts_base = np.zeros([4,4,4])
checkpoints_raw = np.zeros([4,3])
img = cv2.imread(os.path.join(filefolder, 'calibration_files', 'four_corners.jpg'))

filename = os.path.join(filefolder, 'calibration_files', 'transforms.yaml')
with open(filename, 'r') as yaml_file:
    data = yaml.load(yaml_file)
#三个变换矩阵（机械臂到世界，世界到相机，相机到图像）
baseToworld_matrix = np.array(data['H_BaseToworld']['data']).reshape(4, 4)
worldTocam_matrix = np.array(data['H_worldTocamera']['data']).reshape(4, 4)
camToimg_matrix = np.array(data['camera_matrix']['data']).reshape(3, 3)
zero3 = np.zeros((3,1))
camToimg_matrix = np.hstack((camToimg_matrix,zero3))
#计算得到检验时四个探针顶点在机械臂基座下的坐标
for i in range(4):
    Tpts_base[i,:,:] = np.dot(pose2matrix(poseeef_base[i,:]), Ttip_eef)
    checkpoints_raw[i,:]=Tpts_base[i,0:3,3].reshape(1,3)

print(poseeef_base)
ones = np.ones([4,1])
#计算得到四个检查点在机械臂坐标系下的齐次坐标
checkpoints = np.hstack((checkpoints_raw,ones))
#在世界坐标系下的坐标
print(checkpoints)
checkpoints_world = np.dot(baseToworld_matrix,checkpoints.T)

#在相机坐标系下的坐标
point_cam = np.dot(worldTocam_matrix,checkpoints_world)
#在图像坐标系中的像素坐标
point_img = np.dot(camToimg_matrix,point_cam/point_cam[2,0])

cv2.circle(img,(int(point_img[0,0]),int(point_img[1,0])),4,(0,  0,  255),-1)
cv2.circle(img,(int(point_img[0,1]),int(point_img[1,1])),4,(0,  0,  255),-1)
cv2.circle(img,(int(point_img[0,2]),int(point_img[1,2])),4,(0,  0,  255),-1)
cv2.circle(img,(int(point_img[0,3]),int(point_img[1,3])),4,(0,  0,  255),-1)
cv2.imwrite(os.path.join(filefolder, 'calibration_files', "check_base_camera.jpg"),img)