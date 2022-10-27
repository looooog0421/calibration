#!/usr/bin/python
# -*- coding: utf-8 -*-
#该标定检测程序的思路是，输入一个棋盘格内部点的世界坐标，使用标定生成的变换矩阵，将这个点变换到相机坐标系再变换到图像坐标系，
#在图片中将这一点显示出来。如果是选择角点的话，则再用生成的像素坐标与标定时保存的对应角点坐标进行对比，输出像素差值
import numpy as np
import cv2
import yaml
import os

filefolder = os.path.dirname(os.path.dirname(__file__)) #上级文件夹
filename = os.path.join(filefolder, 'calibration_files', 'transforms.yaml')
with open(filename, 'r') as yaml_file:
    data = yaml.load(yaml_file)
camToworld_matrix = np.array(data['H_worldTocamera']['data']).reshape(4, 4)
imgTocam_matrix = np.array(data['camera_matrix']['data']).reshape(3, 3)

img = cv2.imread(os.path.join(filefolder, 'calibration_files', 'four_corners.jpg'))

zero3 = np.zeros((3,1))
imgTocam_matrix = np.hstack((imgTocam_matrix,zero3))

point = np.array([0.04, 0.08, 0., 1.]).reshape(4,1)#此处输入check点的齐次世界坐标

point_cam = np.dot(camToworld_matrix,point)
point_img = np.dot(imgTocam_matrix,point_cam/point_cam[2,0])

cv2.circle(img,(int(point_img[0,0]),int(point_img[1,0])),4,(0,  0,  255),-1)
cv2.imwrite(os.path.join(filefolder, 'calibration_files', 'check_world_camera.jpg'),img)
#如果选择的是角点，则加一步计算像素偏差
corners = np.load(os.path.join(filefolder, 'calibration_files', 'corners.npy'))
i = 7 # 此处根据选择的角点在corners内的索引值修改
check = np.array([point_img[:2,0].reshape(1,2)-corners[i,:,:]]).reshape(2,1)
print("check_result = ",check)