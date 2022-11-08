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
    image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    image = vibrance(image, 40)
    image = brightenShadows(image, 40)
    image = cv.cvtColor(image, cv.COLOR_HSV2RGB)

    image_H = image[:, :, 0]
    cv.applyColorMap(image_H, cv.COLORMAP_JET)

    cv.imshow("dst", image_H)
    cv.waitKey(0)

    pixel_value = np.float32(image_H.reshape((-1, 3)))

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
    segmented_image = segmented_image.reshape((image_H.shape))

    cv.imshow("", segmented_image)
    cv.waitKey(0)



if __name__ == '__main__':
    abs_path=os.path.dirname(os.path.abspath(__file__))
    # color_raw = o3d.io.read_image(os.path.join(abs_path, "images", "color_1.png"))
    test_KMeans(os.path.join(abs_path, "images", "color_image.png"))