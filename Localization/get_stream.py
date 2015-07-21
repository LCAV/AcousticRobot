#-*- coding: utf-8 -*-
import cv2 
import urllib
import numpy as np

k = 0
#stream = urllib.urlopen("http://172.16.156.139:8080/frame.mjpg")
#stream = open('test.mjpg','rb')
stream = urllib.urlopen("http://172.16.156.139:8080/?action=stream")
bytes=''
while True:
	bytes+=stream.read(1024)
	a = bytes.find('\xff\xd8')
	b = bytes.find('\xff\xd9')

	# New image found
	if a!=-1 and b!=-1:
		jpg = bytes[a:b+2]
		bytes = bytes[b+2:]
		i = cv2.imdecode(np.fromstring(jpg,dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
		cv2.imshow('i',i)
		#print(bytes)
		#print(jpg)
		if cv2.waitKey(1)==27:
			exit(0)

