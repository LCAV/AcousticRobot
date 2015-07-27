#-*- coding:utf-8 -*-
from __future__ import division
from multiprocessing import Process

import cv2
import cv2.cv as cv
import numpy as np
import matplotlib.pylab as plt
import time

def resize(img,height):
	r = height / img.shape[0]
	dim = (int(img.shape[1]*r),height)
	return cv2.resize(img,dim,interpolation=cv2.INTER_AREA)

def plotimage(img,height):
	img = resize(img,height)
	plt.imshow(img)
	plt.show(block=False)


height=300
dist,p1,p2,rmin,rmax=(30,10,35,1,30)
# Get test images
img = cv2.imread("pics/room_dots.jpeg",cv2.IMREAD_GRAYSCALE)
h, b_global = cv2.threshold(img,254,255,cv2.THRESH_BINARY)
b_adapt = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,5,0)
b = [img,b_adapt]

plt.close(1)
plt.figure(1,figsize=(10,10))
j = 0
circles=[]
for bs in b:
	j+=1
	# plot raw_images
	plt.subplot(len(b),len(b),j),plt.imshow(bs,'gray')
	# plot circles
	bs_circ=bs.copy()
	circ = cv2.HoughCircles(bs, cv.CV_HOUGH_GRADIENT, 1,dist, np.array([]), p1,p2,rmin,rmax)
	circles.append(circ)
	if circ != None:
		a, num_circ, c = circ.shape
		for i in range(num_circ):
			cv2.circle(bs_circ, (circ[0][i][0], circ[0][i][1]), circ[0][i][2], (0, 0, 255), 3, cv2.CV_AA)
			cv2.circle(bs_circ, (circ[0][i][0], circ[0][i][1]), 2, (0, 255, 0), 3, cv2.CV_AA)

		print(len(b))
		plt.subplot(len(b),len(b),j+len(b)),plt.imshow(bs_circ)

plt.show(block=False)
print(circles)


#img = resize(img,height)





'''
plt.imshow(img)
plt.show(block=False)
counter = 0
try:
	while 1:
		p = Process(target=plotimage, args=(img,height))
		p.start()
		time.sleep(3)
		counter += 1
		if counter > 5:
			break
except KeyboardInterrupt:
	plt.close('all')
	'''
