# -*- coding: utf-8 -*-
import cv2, os, sys, getopt
import numpy as np
import cv2.cv as cv
import matplotlib.pyplot as plt
import urllib

def get_filename():
	inputfile = ''
	try:
		opts,args = getopt.getopt(sys.argv[1:],"i:",["ifile="])
	except getopt.GetoptError:
		print("usage file.py -i <inputfile>")
		sys.exit(2)
	for opt, arg in opts:
		if opt in ("-i","--ifile"):
			try:
				with open(arg,'r+'):
					inputfile = arg
			except:
				print("could not find input file")
				sys.exit(2)
	return inputfile

def get_image():
	img = ''
	not_found = 1
	counter = 1
	bytes=''
	try:
		stream = urllib.urlopen("http://172.16.156.139:8080/?action=stream")
	except:
		print("Could not open stream")
		sys.exit(1)
	while not_found:
		counter = counter+1
		try:
			bytes+=stream.read(1024)
			a = bytes.find('\xff\xd8')
			b = bytes.find('\xff\xd9')
			# New image found
			if a!=-1 and b!=-1:
				jpg = bytes[a:b+2]
				bytes = bytes[b+2:]
				i = cv2.imdecode(np.fromstring(jpg,dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
				not_found=0
				return i
			if counter > 1000:
				print("Timeout while loading stream")
				sys.exit(1)
		except:
			print("Could not load stream")
			sys.exit(1)

class InteractivePlot:
	def __init__(self,pict,name):
		self.x1 = 0.0
		self.y1 = 0.0
		self.x2 = 0.0
		self.y2 = 0.0
		self.x3 = 0.0
		self.y3 = 0.0
		self.x4 = 0.0
		self.y4 = 0.0
		self.pict = pict
		self.name = name
	def setpoints(self):
		print("Choose the 4 indicator points (left to right, top to bottom)")
		self.cid = self.pict.canvas.mpl_connect('button_press_event',self.onclick)
	def onclick(self,event):
		if self.x1==0.0 and self.y1 == 0.0:
			self.x1 = event.xdata
			self.y1 = event.ydata
			print('1: {0:5.2f},{1:5.2f}'.format(self.x1,self.y1))
		elif self.x2==0.0 and self.y2 == 0.0:
			self.x2 = event.xdata
			self.y2 = event.ydata
			print('2: {0:5.2f},{1:5.2f}'.format(self.x2,self.y2))
		elif self.x3==0.0 and self.y3 == 0.0:
			self.x3 = event.xdata
			self.y3 = event.ydata
			print('3: {0:5.2f},{1:5.2f}'.format(self.x3,self.y3))
		elif self.x4==0.0 and self.y4 == 0.0:
			self.x4 = event.xdata
			self.y4 = event.ydata
			print('4: {0:5.2f},{1:5.2f}'.format(self.x4,self.y4))
			plt.close()
	def getpoints(self):
		return self.x1,self.y1,self.x2,self.y2,self.x3,self.y3,self.x4,self.y4
	def draw(self):
		plt.imshow(self.name)
		plt.show()

def get_position(img):
	# Apply some filters to picture
	# Convert to graylevel picture (if not already the case)
	src = img
	# create 1-channel CV_8U image
	if img.dtype != 'uint8':
		img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	ret, thresh = cv2.threshold(img,127,255,cv2.THRESH_BINARY)

	# Get contours (and hierarchy) 
	contours, h = cv2.findContours(thresh,cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
	print(contours)
	if contours == 0:
		print("no contours detected")
	else:
		for cnt in contours:
			approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
			print(len(approx))
			if len(approx)>3:
				print("circle detected with ",len(approx)," vertices")
				cv2.drawContours(img,[cnt],0,(0,255,255))
			else:
				print("no circle detected. virteces: ",len(approx))
	plt.subplot(121),plt.imshow(src),plt.title('RGB')
	plt.subplot(122),plt.imshow(thresh),plt.title('Binary')
	plt.show()
	return 1

def get_position_hough(img):
	src = img
	cimg = src.copy()
	# create 1-channel CV_8U image
	if img.dtype != 'uint8':
		img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY,1)
	img = cv2.medianBlur(img, 5)
	circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1 , 10, np.array([]), 100,30,1,30)
	a, b, c = circles.shape
	for i in range(b):
		cv2.circle(cimg, (circles[0][i][0], circles[0][i][1]), circles[0][i][2], (0, 0, 255), 3, cv2.LINE_AA)
		cv2.circle(cimg, (circles[0][i][0], circles[0][i][1]), 2, (0, 255, 0), 3, cv2.LINE_AA) # draw center of circle
	plt.subplot(121),plt.imshow(src),plt.title('source')
	plt.subplot(122),plt.imshow(cimg),plt.title('detected circles')
	plt.show()
	return 1

def manual_conversion(img):
	# Get indicator points
	img_i = InteractivePlot(plt.figure(),img)
	img_i.setpoints()
	img_i.draw()
	x1,y1,x2,y2,x3,y3,x4,y4 = img_i.getpoints()

	# Map to real image
	xreal = 400
	yreal = 200
	pts1 = np.float32([[x1,y1],[x2,y2],[x3,y3],[x4,y4]])
	pts2 = np.float32([[0,0],[xreal,0],[0,yreal],[xreal,yreal]])

	M = cv2.getPerspectiveTransform(pts1,pts2)
	img_flat = cv2.warpPerspective(img,M,(xreal,yreal))
	return img_flat

def main():
	while True:
		try:
			choice = raw_input("Do you want to perfrom a localisation? (y/n)")
			if choice == "y":
				# Close previous plots
				plt.close('all')

				# Read image
				inputfile=get_filename()
				if inputfile == '':
					img = get_image()
				else:
					img = cv2.imread(get_filename(),cv.CV_LOAD_IMAGE_GRAYSCALE)
					if img == None:
						inputfile=("Could not open file, enter filename (or q to quit)")
						if inputfile == "q":
							sys.exit(1)
				#rows,cols,ch = img.shape
				#print(rows,cols,ch)

				#img_flat = manual_conversion(img)
				img_flat = cv2.imread('test.jpg',cv2.IMREAD_GRAYSCALE)
				# Find robot position
				position = get_position(img_flat)
				position2 = get_position_hough(img_flat)
				plt.subplot(121),plt.imshow(img),plt.title('Input')
				plt.subplot(122),plt.imshow(img_flat),plt.title('Output')
				plt.show()
			elif choice == "n":
				sys.exit(1)
		except (KeyboardInterrupt, SystemExit):
			print("Program terminated by user")
			sys.exit(1)
	return 0

main()
