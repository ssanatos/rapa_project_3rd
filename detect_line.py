import cv2 as cv

gray_img = cv.imread('./images/line.png',cv.IMREAD_GRAYSCALE)
cv.imshow('gray_img',gray_img)

rect_img = cv.rectangle(gray_img, (626, 771), (702, 864), (0,0,255), 3)
rect_img = cv.rectangle(gray_img, (895, 788), (983, 863), (0,0,255), 3)
cv.imshow('gray_img',rect_img)

left_img = gray_img[626:702, 771:864]
right_img = gray_img[865:983, 788:863]

import numpy as np

if np.all(left_img < 90):
    print('Right')
    pass
if np.all(right_img < 90):
    print('Left')
    pass

cv.waitKey()
pass