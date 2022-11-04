import os
import cv2
import numpy as np


abs_path=os.path.dirname(os.path.abspath(__file__))
img=cv2.imread(os.path.join(abs_path, "images", "color_image.png"))
img=cv2.resize(src=img,dsize=(450,450))
#图像分割
dst=cv2.pyrMeanShiftFiltering(src=img,sp=20,sr=30)
#图像分割(边缘的处理）
canny=cv2.Canny(image=dst,threshold1=30,threshold2=100)
#查找轮廓
conturs,hierarchy=cv2.findContours(image=canny,mode=cv2.RETR_EXTERNAL,method=cv2.CHAIN_APPROX_SIMPLE)
#画出轮廓
cv2.drawContours(image=img,contours=conturs,contourIdx=-1,color=(0,255,0),thickness=3)

cv2.imshow('img',img)
cv2.imshow('dst',dst)
cv2.imshow('canny',canny)
cv2.waitKey(0)
cv2.destroyAllWindows()

if __name__ == '__main__':
    print('Pycharm')