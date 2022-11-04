import cv2 as cv
import numpy as np

img = cv.imread("color_image.png", flags=0)
cv.imshow("", img)
cv.waitKey(0)