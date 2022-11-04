#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import numpy as np
import cv2 as cv
import os



def test_KMeans(image_path) :
    image = cv.imread(image_path, cv.IMREAD_COLOR)
    image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
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

    cv.imshow("", segmented_image)
    cv.waitKey(0)
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
