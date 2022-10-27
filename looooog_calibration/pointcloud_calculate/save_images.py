#!/usr/bin python
# -*- coding: utf-8 -*-
import os
import png
import time
import rospy

import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge,CvBridgeError

Abs_Path=os.path.dirname(os.path.abspath(__file__))

class Capture:
    def __init__(self,init_node):
        if init_node:
            rospy.init_node("Capture")
        self.bridge=CvBridge()


    def get_image(self):
        image=rospy.wait_for_message('/camera/color/image_raw',Image,timeout=3) #"等待来自image_raw的图像信息"
        cv_image=self.bridge.imgmsg_to_cv2(image,"bgr8") #"将ros图像转换为openCV的图像"
        return cv_image

    def get_depth(self):
        image=rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw',Image,timeout=3)
        depth_image=self.bridge.imgmsg_to_cv2(image,"32FC1")
        return depth_image

    def get_color_map(self,depth_image=None,range=None):
        """
        送入深度图,返回对应的颜色图
        :param depth_image:需要生成的颜色图,如果为None,则选取自带的深度图
        :param range: 是否需要滤除掉一定距离之后的值
        :return:
        """
        #没有深度图则直接采用类中原本的深度图
        if depth_image is None:
            depth_image=self.depth_image

        #有range要求则进行阈值操作
        range_image=depth_image.copy()
        if range is not None:
            depth_mask=cv.inRange(depth_image,0,range)
            range_image=depth_image*depth_mask/255

        #开始转深度图
        color_map=range_image.copy()
        cv.normalize(color_map,color_map,255,0,cv.NORM_MINMAX)
        color_map=color_map.astype(np.uint8)
        color_map=cv.applyColorMap(color_map,cv.COLORMAP_JET)
        self.color_map=color_map

        return color_map



def see_image():
    """
    To see image from realsense.if want to save image,press 's'
    This may help when make template
    :return:
    """
    capture=Capture(init_node=True)
    image_number=0
    Abs_Path=os.path.dirname(os.path.abspath(__file__))   #取当前目录的绝对路径并去掉文件名，即获取程序所在文件夹的绝对路径
    save_path=os.path.join(Abs_Path,"images") #在当前所在文件夹下创建新文件夹
    while not rospy.is_shutdown():                  #判断节点是否关闭
        #get_image example
        image=capture.get_image()
        depth_image=capture.get_depth()
        # print("min is {}".format(np.min(depth_image)))
        depth_color_map=capture.get_color_map(depth_image)
        cv.imshow("Image",image) #"显示图像"
        cv.imshow("color_map",depth_color_map) #"显示图像"
        cv.waitKey(3) #"图像延时"
        image=capture.get_image()
        cv.imshow("Image",image)
        
        #Save image
        data=cv.waitKey(700)
        if data==ord('s'):
            if not os.path.exists(save_path):
                os.mkdir(save_path)
            color_name="color_{}.png".format(image_number)
            depth_name="depth_{}.png".format(image_number)
            cv.imwrite(os.path.join(save_path,color_name),image)

            with open(os.path.join(save_path,depth_name), 'wb') as f:#对于png文件,专门采用了png文件的保存方式
                writer = png.Writer(width=depth_image.shape[1], height=depth_image.shape[0],
                                    bitdepth=16, greyscale=True)
                zgray2list = depth_image.tolist()
                writer.write(f, zgray2list)

            
            print("save image:{}".format(color_name))
            
            image_number=image_number+1
        if data==ord('q'):
            break

def generate_mask():
    save_path=os.path.join(Abs_Path,"images")
    # for index in range(16):
    if True:
        index=10
        color_path=os.path.join(save_path,"color_{}.png".format(index))
        depth_path=os.path.join(save_path,"depth_{}.png".format(index))

        color_image=cv.imread(color_path)
        depth_image=cv.imread(depth_path,cv.IMREAD_UNCHANGED)

        ROI=np.zeros(color_image.shape)
        ROI[90:380,190:400]=1

        depth_ROI=np.zeros(depth_image.shape)
        depth_ROI[90:380,190:400]=1
        
        cv.imshow("color",color_image)

        mask_image=color_image*ROI
        mask_image=mask_image.astype(np.uint8)

        depth_image=depth_image*depth_ROI
        print(np.max(depth_image))
        depth_image[depth_image==0]=1000
        depth_range=cv.inRange(depth_image,0,730)
        cv.imshow("depth_range",depth_range)
        print(depth_range.shape)
        

        save_seg_image=np.array([depth_range,depth_range,depth_range])
        print(save_seg_image.shape)
        save_seg_image=save_seg_image.transpose([1,2,0]).astype(np.uint8)
        print(save_seg_image.shape)
        cv.imwrite(os.path.join(save_path,"segmask_{}.png".format(index)),save_seg_image)

        cv.imshow("mask",mask_image)

        cv.waitKey(0)

def generate_by_color():
    save_path=os.path.join(Abs_Path,"images")
    for index in range(16):
    # if True:
    #     index=10
        color_path=os.path.join(save_path,"color_{}.png".format(index))
        depth_path=os.path.join(save_path,"depth_{}.png".format(index))

        color_image=cv.imread(color_path)
        depth_image=cv.imread(depth_path,cv.IMREAD_UNCHANGED)

        

        ROI=np.zeros(color_image.shape)
        ROI[90:380,190:400]=1

        depth_ROI=np.zeros(depth_image.shape)
        depth_ROI[90:380,190:400]=1
        
        cv.imshow("color",color_image)

        mask_image=color_image*ROI
        mask_image=mask_image.astype(np.uint8)
        hsv_image=cv.cvtColor(mask_image,cv.COLOR_BGR2HSV)
        cv.imshow("hsv_image",hsv_image)

        depth_image=depth_image*depth_ROI
        print(np.max(depth_image))
        depth_image[depth_image==0]=1000
        depth_range=cv.inRange(depth_image,0,730)
        cv.imshow("depth_range",depth_range)
        print(depth_range.shape)
        

        save_seg_image=np.array([depth_range,depth_range,depth_range])
        print(save_seg_image.shape)
        save_seg_image=save_seg_image.transpose([1,2,0]).astype(np.uint8)
        print(save_seg_image.shape)
        # cv.imwrite(os.path.join(save_path,"segmask_{}.png".format(index)),save_seg_image)

        cv.imshow("mask",mask_image)

        cv.waitKey(0)

def convert_depth_to_npy():
    # for index in range(16):
    if True:
        index=10
        print(os.path.join(Abs_Path,"images/depth_{}.png".format(index)))
        depth_image=cv.imread(os.path.join(Abs_Path,"images/depth_{}.png".format(index)),cv.IMREAD_UNCHANGED)
        print(depth_image.shape)
        print(depth_image.dtype)
        depth_image=depth_image.astype(np.float32)/1000.0
        depth_image[depth_image==0]=0.1
        print(np.max(depth_image))
        print(np.min(depth_image))
        np.save(os.path.join(Abs_Path,"images/depth_{}.npy").format(index),depth_image)



if __name__ == "__main__":
    see_image()
    # generate_mask()
    # generate_by_color()
    # convert_depth_to_npy()
    
    



