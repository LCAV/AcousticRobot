#!/usr/bin/python
# -*- coding: utf-8 -*-

'''
This example illustrates how to use cv2.HoughCircles() function.
Usage: ./houghcircles.py [<image_name>]
image argument defaults to ../data/board.jpg
'''

import cv2
import cv2.cv as cv
import numpy as np
import sys
import getopt
import matplotlib.pyplot as plt

class MyParameters:
	def __init__(self):
		self.fn = "test2.jpg"
		self.dist = 30
		self.p1 = 10
		self.p2 = 30
		self.rmin = 1
		self.rmax = 40
		try:
			opts, args = getopt.getopt(sys.argv[1:], "i:p1:p2:min:max:dist:")
		except getopt.GetoptError:
			print("usage file.py -i <inputfile> -p1 <param1> -p2 <param2>")
			print("param1 is te upper threshold for Canny edge detection")
			print("param2 is threshold for center detection")
			print("-min <minimum radius> -max <maximumradius>") 
			print("-dist <mininum distance>")
			sys.exit(2)
		for opt, arg in opts:
			if opt == "-i":
				if cv2.imread(arg,0).any():
					self.fn=arg
				else:
					print("could not read input file")
					sys.exit(2)
			elif opt == "-p1":
				self.p1 = arg
			elif opt == "-p2":
				self.p2 = arg
			elif opt == "-min":
				self.rmin = arg
			elif opt == "max":
				self.rmax=arg
			elif opt == "dist":
				self.dist=arg

# Find circles
def circle_method(img,cimg,p):
	circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1, p.dist, np.array([]), p.p1, p.p2, p.rmin, p.rmax)
	a, b, c = circles.shape
	for i in range(b):
		cv2.circle(cimg, (circles[0][i][0], circles[0][i][1]), circles[0][i][2], (0, 0, 255), 3, cv2.CV_AA)
		cv2.circle(cimg, (circles[0][i][0], circles[0][i][1]), 2, (0, 255, 0), 3, cv2.CV_AA) # draw center of circle

	plt.subplot(222),plt.imshow(cimg),plt.title("detected circles")

# Find smallest contour
def contour_method(img):
	h, img = cv2.threshold(img,157,255,cv2.THRESH_BINARY)
	plt.subplot(223),plt.imshow(img),plt.title("binary")

	contours = cv2.findContours(img,mode=cv2.RETR_EXTERNAL,method=cv2.CHAIN_APPROX_SIMPLE)[0]
	areas=[cv2.contourArea(ctr) for ctr in contours]
	min_contour = [contours[areas.index(min(areas))]]
	print(min_contour)

def resize(img,height):
	r = height/img.shape[0]
	dim = (int(image.shape[1]*r,height))
	return resized(cv2.resize(img,dim,interpolation=cv2.INTER_AREA)


def main():
	p = MyParameters()

	src = cv2.imread(p.fn,cv2.IMREAD_GRAYSCALE)
	cimg = src.copy() # numpy function

	# Primary filtering
	img = cv2.medianBlur(src, 5)
	img = cv2.GaussianBlur(src, (5,5),4)
	circle_method(img,cimg,p)
	contour_method(img)

	plt.subplot(221),plt.imshow(src),plt.title("source")
	plt.show(block=False)

go_on = 1
while go_on:
	main()
	go_on = int(raw_input("continue? (1/0)"))

print("Program stopped manually")
sys.exit(1)
