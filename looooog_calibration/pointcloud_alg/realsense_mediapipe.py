#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from re import A
from select import select
from socket import SO_KEEPALIVE
from symbol import testlist_comp
from time import clock_getres
from typing_extensions import Self
import cv2
import mediapipe as mp
import pyrealsense2 as rs
import sys
from unicodedata import name
import numpy as np

def Align_version(frames, align, show_pic=0):

    # 对齐版本
    aligned_frames = align.process(frames)
    depth_frame_aligned = aligned_frames.get_depth_frame()
    color_frame_aligned = aligned_frames.get_color_frame()
    # if not depth_frame_aligned or not color_frame_aligned:
    #     continue
    color_image_aligned = np.asanyarray(color_frame_aligned.get_data())
    depth_image_aligned = np.asanyarray(depth_frame_aligned.get_data())
 
    depth_colormap_aligned = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_aligned, alpha=0.05), cv2.COLORMAP_JET)
    images_aligned = np.hstack((color_image_aligned, depth_colormap_aligned))
    if show_pic:
        cv2.imshow('aligned_images', images_aligned)
    return color_image_aligned,depth_image_aligned,depth_colormap_aligned


class realsenseMediapipe:

    def __init__(self) -> None:
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_hands = mp.solutions.hands
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        

    def Normalize_landmarks(self, image, hand_landmarks):
        new_landmarks = []
        
        for i in range(0,len(hand_landmarks.landmark)):
            float_x = hand_landmarks.landmark[i].x
            float_y = hand_landmarks.landmark[i].y
            # Z坐标靠近屏幕增大，远离屏幕减小
            float_z = hand_landmarks.landmark[i].z
            # print(float_z)
            width = image.shape[1]
            height = image.shape[0]
            
            pt = self.mp_drawing._normalized_to_pixel_coordinates(float_x,float_y,width,height)
            new_landmarks.append(pt)
        return new_landmarks

    def InitCamera(self):
        #初始化了两个数据流类型(深度图和彩色图)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        #左右双目
        self.config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)  
        self.config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)

        # Start streaming
        # #pipeline为程序与摄像头交互的一个通讯模型,可以理解在对realsense操作时的一个必要操作(初始化)
        self.pipeline.start(self.config)

        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = self.pipeline.wait_for_frames() #获取摄像头的实时帧

            # depth_frame = frames.get_depth_frame() #从实时帧中获取深度图像
            # color_frame = frames.get_color_frame() #从实时帧中获取彩色图像
            color_image, depth_image, depth_colormap=Align_version(frames,show_pic=True)
            # if not depth_frame or not color_frame:
            #     continue #确保深度图和彩色图都获得成功

            # # Convert images to numpy arrays 把图像转换为numpy data
            # depth_image = np.asanyarray(depth_frame.get_data()) #从帧中获取数据
            # color_image = np.asanyarray(color_frame.get_data())
            print(depth_image.shape)
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first) 在深度图上用颜色渲染
            # convertScaleAbs可以对src中每一个元素做
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            
            with self.mp_hands.Hands(
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
                        self.mp_drawing.draw_landmarks(
                            color_image,
                            hand_landmarks,
                            self.mp_hands.HAND_CONNECTIONS,
                            self.mp_drawing_styles.get_default_hand_landmarks_style(),
                            self.mp_drawing_styles.get_default_hand_connections_style())
                        # print(hand_landmarks)
                        # print(type(hand_landmarks))
                        # np.savetxt("hand_landmarks.txt",hand_landmarks)
                        self.skeleton_points = self.Normalize_landmarks(color_image,hand_landmarks)
                        
                images = np.hstack((depth_colormap, color_image))
                # print(skeleton_points)
                
                # cv2.imshow('MediaPipe Hands', images)
                # if cv2.waitKey(5) & 0xFF ==27:
                #     break            


if __name__ == "__main__":
    testClass = realsenseMediapipe()
    testClass.InitCamera()
    skeleton = testClass.skeleton_points
    print(skeleton)
