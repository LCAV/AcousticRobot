#-*- coding: utf-8 -*-
##@package get_image
#
# Simple module to read image from specified webcam.

from __future__ import print_function
import urllib.request
import cv2
import numpy as np

def get_image(n):
    ''' Get the current webcam image from IP-stream '''
    img = ''
    not_found = 1
    counter = 1
    bytes=''
    stream =  ''
    stream = urllib.request.urlopen("http://172.16.156."+str(n)+":8080/?action=stream")
    while not_found:
        counter = counter+1
        bytes+=stream.read(1024)
        a = bytes.find('\xff\xd8')
        b = bytes.find('\xff\xd9')
        # New image found
        if a!=-1 and b!=-1:
            jpg = bytes[a:b+2]
            bytes = bytes[b+2:]
            i = cv2.imdecode(np.fromstring(jpg,dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
            print("Image loaded:",i.shape)
            not_found=0
            return i
        if counter > 10000:
            print("Timeout while loading stream")

if __name__ == '__main__':
    print("get image from stream")
    get_image(139)
