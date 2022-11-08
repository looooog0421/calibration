#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import numpy as np
import cv2 as cv
import os

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

def test_KMeans(image_path) :
    image = cv.imread(image_path, cv.IMREAD_COLOR)
    # image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
    image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    image = vibrance(image, 40)
    image = brightenShadows(image, 40)
    image = cv.cvtColor(image, cv.COLOR_HSV2RGB)
    cv.imshow("src", image)
    pixel_value = np.float32(image.reshape((-1, 3)))

    #终止条件
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 200, 0.1)

    #起始的中心选择
    flags = cv.KMEANS_RANDOM_CENTERS

    #定义簇的数量
    K = 3

    _, labels, center = cv.kmeans(pixel_value, K, None, criteria, 10, flags)
    center = np.uint8(center)

    #将所有像素转换为质心的颜色
    segmented_image = center[labels.flatten()]
    
    #重塑回原始图像尺寸
    segmented_image = segmented_image.reshape((image.shape))

    gray = cv.cvtColor(segmented_image,cv.COLOR_BGR2GRAY)
    ret,b = cv.threshold(gray,60,255,cv.THRESH_BINARY)
    contours,h = cv.findContours(b,cv.RETR_LIST,\
                                cv.CHAIN_APPROX_SIMPLE)
    print(len(contours))
    x = 0
    for i in range(len(contours)):
        area = cv.contourArea(contours[i])
        print(area)
        if area > 10000:
            x = i
    cnt = contours[x]
    a = cv.approxPolyDP(cnt,1,True)
    adp = segmented_image.copy()
    segmented_image = cv.polylines(adp,[a],True,(255,0,0),2)
    cv.imshow('img1',segmented_image)
    cv.waitKey()
    cv.destroyAllWindows()

    # cv.imshow("", segmented_image)
    # cv.waitKey(0)
    # plt.figure(figsize = (8, 4))
    # plt.subplot(121)
    # plt.imshow(image)
    # plt.axis('off')
    # plt.title(f'$input\_image$')
    # plt.subplot(122)
    # plt.imshow(segmented_image)
    # plt.axis('off')
    # plt.title(f'$segmented\_image$')
    # plt.tight_layout()
    # plt.savefig('segmented_result.png')
    # plt.show()

if __name__ == '__main__':
    abs_path=os.path.dirname(os.path.abspath(__file__))
    # color_raw = o3d.io.read_image(os.path.join(abs_path, "images", "color_1.png"))
    test_KMeans(os.path.join(abs_path, "images", "color_image.png"))
