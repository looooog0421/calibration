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


def Normalize_landmarks(image, hand_landmarks, depth):
    new_landmarks = []
    for i in range(0,len(hand_landmarks.landmark)):
        float_x = hand_landmarks.landmark[i].x
        float_y = hand_landmarks.landmark[i].y
        
        width = image.shape[1]
        height = image.shape[0]
        pt = mp_drawing._normalized_to_pixel_coordinates(float_x,float_y,width,height)
        
        if pt != None:
            pt = list(pt)
            float_z = depth[int(float_x),int(float_y)]
            # print(float_z)
            pt.append(float_z)
        new_landmarks.append(pt)
    print(new_landmarks[1])
    return new_landmarks


mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

pipeline = rs.pipeline()
config = rs.config()

#初始化了两个数据流类型(深度图和彩色图)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
 

# Start streaming
# #pipeline为程序与摄像头交互的一个通讯模型,可以理解在对realsense操作时的一个必要操作(初始化)
pipeline.start(config) 


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
    # print(depth_image.shape)
    # Apply colormap on depth image (image must be converted to 8-bit per pixel first) 在深度图上用颜色渲染
    # convertScaleAbs可以对src中每一个元素做
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    
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
                # np.savetxt("hand_landmarks.txt",hand_landmarks)
                skeleton_points = Normalize_landmarks(color_image,hand_landmarks,depth_image)
                print(skeleton_points)
                
        images = np.hstack((depth_colormap, color_image))
        # print(skeleton_points)
        
        cv2.imshow('MediaPipe Hands', cv2.flip(images,1))
        if cv2.waitKey(5) & 0xFF ==27:
            break
