# simple way to detect the rectangle inside an image

import cv2
import numpy as np
from matplotlib import pyplot as plt
#import zbarlight

img = cv2.imread('test3.jpg',0)
img2 = img.copy()
template = cv2.imread('qr.jpg',0)

height, width = img2.shape[:2]
#print('width is ',width, 'height is', height)
h, w = template.shape[:2]
#w,h = template.shape[::-1]

methods = ['cv2.TM_CCOEFF']
#other methods like 'cv2.TM_CCORR','cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED'

for meth in methods:
    img = img2.copy()
    method = eval(meth)

    # Apply template Matching
    res = cv2.matchTemplate(img,template,method)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

    top_left = max_loc
    bottom_right = (top_left[0] + w, height - top_left[1] - h)
    top_left = (top_left[0], height - top_left[1])
    middle_point = (top_left[0]+w/2,top_left[1]+h/2)

    print('The size of the image is height= ', height,'width= ',width)
    print('Top left point is at ',top_left,'bottom right point is at', bottom_right,'and centroid is at', middle_point)

    a=cv2.rectangle(img,top_left, bottom_right, 255, 2)

    plt.subplot(121),plt.imshow(res,cmap = 'gray')
    plt.title('Matching Result'), plt.xticks([]), plt.yticks([])

    plt.subplot(122),plt.imshow(img,cmap = 'gray')
    plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
    plt.suptitle(meth)

    plt.show()