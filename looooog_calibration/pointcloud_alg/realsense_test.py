#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import sys
from unicodedata import name
import pyrealsense2 as rs
import numpy as np
import cv2
# Configure depth and color streams 
pipeline = rs.pipeline()
config = rs.config()

#初始化了两个数据流类型(深度图和彩色图)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
 

# Start streaming
# #pipeline为程序与摄像头交互的一个通讯模型,可以理解在对realsense操作时的一个必要操作(初始化)
pipeline.start(config) 
 
def vibrance(img, amount):
    """
    this function is an approximated implementation for vibrance filter by Photoshop that increases the saturation of
    an image in a way that the increasing amount for the low saturated pixels is more than the increasing amount for
    pixels that are already saturated
    Parameters:
        img (ndarray): input image in HSV color space
        amount (int): increasing vibrance amount
    Returns:
         image in HSV color space after applying vibrance filter
    """
    amount = min(amount, 100)
    sat_increase = ((255 - img[:, :, 1]) / 255 * amount).astype(np.uint8)
    img[:, :, 1] += sat_increase
    return img

def brightenShadows(img, amount):
    """
    this function increases the brightness of the dark pixels of an image
    Parameters:
        img (ndarray): input image in HSV color space
        amount (int): increasing brightness amount
    Returns:
         image in HSV color space after applying brightness filter
    """
    amount = min(amount, 100)
    val_inc = ((255 - img[:, :, 2]) / 255 * amount).astype(np.uint8)
    img[:, :, 2] += val_inc
    return img

# try:
while True:
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames() #获取摄像头的实时帧
    
    depth_frame = frames.get_depth_frame() #从实时帧中获取深度图像
    color_frame = frames.get_color_frame() #从实时帧中获取彩色图像
    
    if not depth_frame or not color_frame:
        continue #确保深度图和彩色图都获得成功

    # Convert images to numpy arrays 把图像转换为numpy data
    depth_image = np.asanyarray(depth_frame.get_data()) #从帧中获取数据
    color_image = np.asanyarray(color_frame.get_data())

    color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    color_image = vibrance(color_image, 45)
    color_image = brightenShadows(color_image, 40)
    color_image = cv2.cvtColor(color_image, cv2.COLOR_HSV2BGR)
    
    # Apply colormap on depth image (image must be converted to 8-bit per pixel first) 在深度图上用颜色渲染
    # convertScaleAbs可以对src中每一个元素做
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    
    # Stack both images horizontally 把两个图片水平拼在一起
    images = np.hstack((color_image, depth_colormap))

    cv2.namedWindow(winname='RealSense',flags=cv2.WINDOW_AUTOSIZE) #设置视窗,flag为表示是否自动设置或调整窗口大小,WINDOW_AUTOSIZE即为自适应
    cv2.imshow('RealSense', images)

    key = cv2.waitKey(1)
    if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
        break
# finally:
#     # Stop streaming
#     pipeline.stop()