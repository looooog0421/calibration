#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
# import numpy as np
# import cv2 as cv

# depth_image = np.load('/home/lgx/logsCalibration/looooog_calibration/pointcloud_alg/depth_image.npy')
# print(depth_image[226, 161])
# np.savetxt("depth_image1.txt",depth_image, fmt='%d')
# cv.imshow(" ", depth_image)
# cv.waitKey(0)

# list1 = [1,2,2,2,3,3]
# list1.remove(0)
# print(list1)

#coding=utf-8
import pyrealsense2 as rs
import numpy as np
import cv2
 
#0:使用相机
# 1:使用API录制好的bag
USE_ROS_BAG=0
#0:彩色图像对齐到深度图;
# 1:深度图对齐到彩色图像
ALIGN_WAY=1
 
def Align_version(frames,align,show_pic=0):
    # 对齐版本
    aligned_frames = align.process(frames)
    depth_frame_aligned = aligned_frames .get_depth_frame()
    color_frame_aligned = aligned_frames .get_color_frame()
    # if not depth_frame_aligned or not color_frame_aligned:
    #     continue
    color_image_aligned = np.asanyarray(color_frame_aligned.get_data())
    if USE_ROS_BAG:
        color_image_aligned=cv2.cvtColor(color_image_aligned,cv2.COLOR_BGR2RGB)
    depth_image_aligned = np.asanyarray(depth_frame_aligned.get_data())
 
    depth_colormap_aligned = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_aligned, alpha=0.05), cv2.COLORMAP_JET)
    images_aligned = np.hstack((color_image_aligned, depth_colormap_aligned))
    if show_pic:
        cv2.imshow('aligned_images', images_aligned)
    return color_image_aligned,depth_image_aligned,depth_colormap_aligned
 
# def Unalign_version(frames,show_pic=0):
#     # 未对齐版本
#     # Wait for a coherent pair of frames: depth and color
#     frames = pipeline.wait_for_frames()
#     depth_frame = frames .get_depth_frame()
#     color_frame = frames .get_color_frame()
 
#     if not USE_ROS_BAG:
#         left_frame = frames.get_infrared_frame(1)
#         right_frame = frames.get_infrared_frame(2)
#         left_image = np.asanyarray(left_frame.get_data())
#         right_image = np.asanyarray(right_frame.get_data())
#         if show_pic:
#             cv2.imshow('left_images', left_image)
#             cv2.imshow('right_images', right_image)
#     # if not depth_frame or not color_frame:
#     #     continue
#     color_image = np.asanyarray(color_frame.get_data())
#     print("color:",color_image.shape)
#     depth_image= np.asanyarray(depth_frame.get_data())
#     print("depth:",depth_image.shape)
 
#     #相机API录制的大小rosbag的rgb图像与depth图像不一致，用resize调整到一样大
#     if USE_ROS_BAG:
#         color_image=cv2.cvtColor(color_image,cv2.COLOR_BGR2RGB)
#         if ALIGN_WAY:  #深度图对齐到彩色图像
#             depth_image=cv2.resize(depth_image,(color_image.shape[1],color_image.shape[0]))
#         else:   #彩色图像对齐到深度图
#             color_image=cv2.resize(color_image,(depth_image.shape[1],depth_image.shape[0]))
#     # 上色
#     depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.05), cv2.COLORMAP_JET)
#     # Stack both images horizontally
#     images = np.hstack((color_image, depth_colormap))
#     if show_pic:
#         cv2.imshow('images', images)
#     return color_image,depth_image,depth_colormap
 
if __name__ == "__main__":
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
 
    if USE_ROS_BAG:
        config.enable_device_from_file("666.bag")#这是打开相机API录制的视频
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  #10、15或者30可选,20或者25会报错，其他帧率未尝试
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        #左右双目
        config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)  
        config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)
 
    if ALIGN_WAY:
        way=rs.stream.color
    else:
        way=rs.stream.depth
    align = rs.align(way)
    profile =pipeline.start(config)
 
 
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("scale:", depth_scale)
    # 深度比例系数为： 0.0010000000474974513
 
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_image_aligned,depth_image_aligned,depth_colormap_aligned=Align_version(frames,align,show_pic=1)
        #     color_image,depth_image,depth_colormap=Unalign_version(frames,show_pic=1)     
            #print(depth_image_aligned*depth_scale)
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        # Stop streaming
        pipeline.stop()
