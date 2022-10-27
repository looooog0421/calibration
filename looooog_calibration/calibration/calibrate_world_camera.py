#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Save the Transform as 4*4 Matrix.
Before Use,Remember to change cb_w,cb_h,cb_size!!
Written by JunnanJiang,2020.12.31.
Log 2020.12.31:
    remove to save H_uw,H_wu,just save the transform between camera and world(checkboard)
    H_bw(Transform between chessboard and world) need to modify to 4*4 in the future
"""
import rospy
import cv2
import numpy as np
import yaml
import os
import random
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

filefolder = os.path.dirname(os.path.dirname(__file__)) #上级文件夹
filename = os.path.join(filefolder, 'calibration_files', 'transforms.yaml')
# corners_filesname = os.path.join(filefolder,'calibration_files', 'corners.yaml')
with open(filename, 'r') as yaml_file:
    data = yaml.load(yaml_file)
K = np.array(data['camera_matrix']['data']).reshape(3, 3)
D = np.array(data['distortion_coefficients']['data']).reshape(1, 5)
K_inv = np.linalg.inv(K)
H_bw = np.array([[0, 1, 0.046], [1, 0, 0.0512], [0, 0, 1]])
H_wb = np.linalg.inv(H_bw)
print('K and D Obtained!\n')

flag_calibration_done = False
def shutdown_node():
    print('This node is shutting down')

# Callback function
def img_callback(img_msg):
    global flag_calibration_done
    if not flag_calibration_done:
        #1: 初始化棋盘格行列数和棋盘角点间隔
        cb_w = 3 
        cb_h = 5
        cb_size = 0.04


        # 生成所有棋盘格角点的坐标
        objpoints = np.zeros((cb_w*cb_h, 3), np.float64)
        objpoints[:, :2] = np.mgrid[0:cb_w, 0:cb_h].T.reshape(-1, 2)
        objpoints = objpoints*cb_size

        # print("objectpoins=",objpoints)  
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # print(cv2.TERM_CRITERIA_EPS)
        Rmatrix = np.zeros((3, 3), np.float64)
        rvecs = np.zeros((3, 1), np.float64) 
        tvecs = np.zeros((3, 1), np.float64) 
        bridge = CvBridge()

        try:
            cv_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        #2: 寻找棋盘格角点在图像坐标系中的坐标
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        cv2.imwrite(os.path.join(filefolder,'calibration_files','raw_img.jpg'),cv_img)
        # cv2.imshow("gray",gray)
        # cv2.waitKey(2)
        
        ret, corners = cv2.findChessboardCorners(cv_img, (cb_w, cb_h), None)
        # print("corners=",corners)
        print(ret)
        if ret != True:
            print('FindChessboardCorners Failed!\n')
            return
        print('Chessboard Corners Found!\n')
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        np.save(os.path.join(filefolder,'calibration_files',"corners.npy"),corners2) #将所有角点的坐标保存在corners.npy文件，以便于后续验证标定结果
        print('Corners Optimiz  ed!\n')


        #2.1：此处将棋盘格四个顶点的坐标提取出来，并在图片中用颜色标注出来，以便于机械臂标定
        four_corners = np.array([corners2[0], corners2[cb_w-1],corners2[cb_w*(cb_h-1)],corners2[cb_w*cb_h-1]])
        four_corners = np.squeeze(four_corners,axis=1)

        cv2.circle(cv_img,(int(four_corners[0,0]),int(four_corners[0,1])),10,(0,  0,  255),-1)
        cv2.circle(cv_img,(int(four_corners[1,0]),int(four_corners[1,1])),10,(0,  255,255),-1)
        cv2.circle(cv_img,(int(four_corners[2,0]),int(four_corners[2,1])),10,(0,  255,0  ),-1) 
        cv2.circle(cv_img,(int(four_corners[3,0]),int(four_corners[3,1])),10,(255,0  ,0  ),-1)
        cv2.imwrite(os.path.join(filefolder,"calibration_files","four_corners.jpg"),cv_img)
        print("image saved!")
        four_corners = four_corners - four_corners[0]
        Ppt_world = np.zeros(four_corners.shape)
        zero = np.zeros((4,1))
        Ppt_world = np.hstack((Ppt_world,zero))

        if(abs(four_corners[1,0])>abs(four_corners[1,1])):
            if(four_corners[1,0]>0):
                Ppt_world[1] = [float(cb_w-1)*cb_size,0.,0.]
            else:
                Ppt_world[1] = [float(1-cb_w)*cb_size,0.,0.]
            
            if(four_corners[3,1]>0):
                Ppt_world[3] = [0.,float(cb_h-1)*cb_size,0.]
            else:
                Ppt_world[3] = [0.,float(1-cb_h)*cb_size,0.]

            Ppt_world[2] = [Ppt_world[1,0],Ppt_world[3,1],0.]
        else:
            if(four_corners[1,1]>0):
                Ppt_world[1] = [0.,float(cb_w-1)*cb_size,0.]
            else:
                Ppt_world[1] = [0.,float(1-cb_w)*cb_size,0.]

            if(four_corners[3,0]>0):
                Ppt_world[3] = [float(cb_h-1)*cb_size,0.,0.]
            else:
                Ppt_world[3] = [float(1-cb_h)*cb_size,0.,0.]
            Ppt_world[2] = [Ppt_world[3,0],Ppt_world[1,1],0.]
        print("four corners saved!")
        # print(four_corners)
        
       
        #3: 计算相机外参矩阵，cw下标表示该矩阵用于世界坐标系变换到相机坐标系
        retval, rvecs, tvecs = cv2.solvePnP(objpoints, corners2, K, D)
        cv2.Rodrigues(rvecs, Rmatrix)
        H_cw=np.eye(4)
        H_cw[:3,:3]=Rmatrix
        H_cw[:3,3]=np.transpose(tvecs)   #world(checkerboard) frame in camera frame
        H_wc = np.linalg.inv(H_cw)

        #4: 保存数据
        RT_data = (""
            + "K_inv:\n"
            + "  rows: 3\n"
            + "  cols: 3\n"
            + "  data: [" + ", ".join(["%8f" % i for i in K_inv.reshape(1,9)[0]]) + "]\n"    
            + "H_worldTocamera:\n"     #The pose of world frame (checkerboard) in camera frame
            + "  rows: 4\n"
            + "  cols: 4\n"
            + "  data: [" + ", ".join(["%8f" % i for i in H_cw.reshape(1,16)[0]]) + "]\n"
            + "H_cameraToworld:\n"     #The pose of camera frame (optical) in world frame
            + "  rows: 4\n"
            + "  cols: 4\n"
            + "  data: [" + ", ".join(["%8f" % i for i in H_wc.reshape(1,16)[0]]) + "]\n"
            + "")
        with open(filename, 'a') as yaml_file:
            yaml_file.write(RT_data)
        flag_calibration_done = True
        print('Calibration Done!\n')
    rospy.on_shutdown(shutdown_node)

# Main function
def main():
    rospy.init_node('easy_worldeye')
    rospy.Subscriber('/camera/color/image_raw', Image, img_callback)
    rospy.spin()

# RUN
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
