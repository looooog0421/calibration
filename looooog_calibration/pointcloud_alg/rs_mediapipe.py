#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from signal import pthread_kill
from socket import SO_KEEPALIVE
from time import clock_getres
import cv2
import mediapipe as mp
import pyrealsense2 as rs
import sys
from unicodedata import name
import numpy as np
import os
from  pointEllipsoid import EllipsoidTool

filefolder = os.path.dirname(os.path.dirname(__file__)) #上级文件夹
filename = os.path.join(filefolder, 'calibration_files', 'depth_image.yaml')

def Align_version(frames,align,show_pic=0):
    # 对齐版本
    aligned_frames = align.process(frames)
    depth_frame_aligned = aligned_frames .get_depth_frame()
    color_frame_aligned = aligned_frames .get_color_frame()
    # if not depth_frame_aligned or not color_frame_aligned:
    #     continue
    color_image_aligned = np.asanyarray(color_frame_aligned.get_data())
    depth_image_aligned = np.asanyarray(depth_frame_aligned.get_data())
 
    depth_colormap_aligned = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_aligned, alpha=0.05), cv2.COLORMAP_JET)
    images_aligned = np.hstack((color_image_aligned, depth_colormap_aligned))
    if show_pic:
        cv2.imshow('aligned_images', images_aligned)
    return color_image_aligned,depth_image_aligned,depth_colormap_aligned

def Normalize_landmarks(image, hand_landmarks, depth):
    new_landmarks = []
    for i in range(0,len(hand_landmarks.landmark)):
        float_x = hand_landmarks.landmark[i].x
        float_y = hand_landmarks.landmark[i].y
        
        width = image.shape[1]
        height = image.shape[0]
        pt = mp_drawing._normalized_to_pixel_coordinates(float_x,float_y,width,height)
        
        # if pt != None:
        #     pt = list(pt)
        #     float_z = depth[int(float_x),int(float_y)]
        #     # print(float_z)
        #     pt.append(float_z)
        new_landmarks.append(pt)
    # new_landmarks = np.array(new_landmarks).reshape(-1,3)
    i = 0
    while i < len(new_landmarks):
        if new_landmarks[i] == None:
            del new_landmarks[i]
        else:
            i += 1
    return new_landmarks


mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

pipeline = rs.pipeline()
config = rs.config()

#初始化了两个数据流类型(深度图和彩色图)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)  
config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30) 

way=rs.stream.color
align = rs.align(way)
# Start streaming
# #pipeline为程序与摄像头交互的一个通讯模型,可以理解在对realsense操作时的一个必要操作(初始化)
profile = pipeline.start(config) 

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

while True:
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames() #获取摄像头的实时帧
    color_image, depth_image, depth_colormap =Align_version(frames,align,show_pic=0)
    # depth_frame = frames.get_depth_frame() #从实时帧中获取深度图像
    # color_frame = frames.get_color_frame() #从实时帧中获取彩色图像
    
    # if not depth_frame or not color_frame:
    #     continue #确保深度图和彩色图都获得成功

    # # Convert images to numpy arrays 把图像转换为numpy data
    # depth_image = np.asanyarray(depth_frame.get_data()) #从帧中获取数据
    # color_image = np.asanyarray(color_frame.get_data())
    # # print(depth_image.shape)
    # # Apply colormap on depth image (image must be converted to 8-bit per pixel first) 在深度图上用颜色渲染
    
    # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    skeleton_points = []
    with mp_hands.Hands(
        model_complexity=0,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as hands:
        success = True
        if not success:
            print("Ignoring empty camera frame.")
            continue
        color_image.flags.writeable = False
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        results = hands.process(color_image)

        color_image.flags.writeable = True
        color_image = cv2.cvtColor(color_image,cv2.COLOR_RGB2BGR)
        # skeleton_points = np.zeros(640*480).reshape(640, 480)
        cv2.imwrite("color_image.png",color_image)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    color_image,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style())
                # print(hand_landmarks)
                # print(type(hand_landmarks))
                
                skeleton_points = Normalize_landmarks(color_image,hand_landmarks,depth_image)
                # print(skeleton_points)
                ellipsoid = EllipsoidTool()
            
        images = np.hstack((depth_colormap, color_image))
        
        # print(len(skeleton_points))
        skeleton_points = np.array(skeleton_points).reshape(-1,2)
        skeleton_depths = np.zeros((skeleton_points.shape[0], 1 ))
        for i in range(skeleton_points.shape[0]):
            skeleton_depths[i] = depth_image[skeleton_points[i, 1], skeleton_points[i, 0]]
            # print(skeleton_points[i])
        skeleton = np.hstack((skeleton_points,skeleton_depths))
        print(skeleton)
        cv2.imshow('MediaPipe Hands', cv2.flip(images,1))
        if cv2.waitKey(5) & 0xFF ==27:
            break
        cv2.imwrite("color_image.png",color_image)
        # np.save("depth_image",depth_image)