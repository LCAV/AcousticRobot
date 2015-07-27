# -*- coding: utf-8 -*-
import future
from future.utils import iteritems
import cv2, os, sys, getopt
import numpy as np
import cv2.cv as cv
import matplotlib.pyplot as plt
import urllib

'''--------------  Basics ------------------ '''
''' Create 1-channel CV_8U image '''
def gray_conversion(img):
    if img.dtype != 'uint8' or img.shape[2] != 1:
        img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    return img
''' Convert BGR (opencv) to RGB (matplotlib) '''
def rgb_conversion(img):
    b,g,r = cv2.split(img)
    return cv2.merge([r,g,b])
''' Get centroid from contour '''
def get_centroid(cnt):
    # Find centroid of contour
    M = cv2.moments(cnt)
    cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
    return cx,cy

'''----------  Program handling ------------ '''
''' Plot where 4 indicator points can be picked by mouse-click'''
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
''' Get filename from command line '''
def get_filename():
    inputfile = ''
    try:
        opts,args = getopt.getopt(sys.argv[1:],"i:",["ifile="])
    except getopt.GetoptError:
        print("usage file.py -i <inputfile>")
        sys.exit(2)
    for opt, arg in opts:
        if opt in ("-i","--ifile"):
            if cv2.imread(arg,0) != None:
                inputfile = arg
            else:
                while cv2.imread(inputfile,0)==None:
                    inputfile=raw_input("Could not open input file, enter valid filename (or q to quit)")
                    if inputfile == "q":
                        sys.exit(2)

    return inputfile
''' Get the current webcam image from IP-stream '''
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

''' --------   Image Processing ------------ '''
''' Circle detection using Contour Polygone vertix counting '''
def get_circles_contours(img):
    img = gray_conversion(img)
    ret, thresh = cv2.threshold(img,200,255,cv2.THRESH_BINARY)

    # Get contours (and hierarchy)
    contours, h = cv2.findContours(thresh,cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

    # Transform picture to 3-channel greyscale for better contour visibility
    img = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

    if contours == 0:
        print("no contours detected")
    else:
        circle_count = 0
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
            if len(approx)>10:
                circle_count+=1
                # print all (-1) contours
                cv2.drawContours(img,[cnt],-1,(0,255,255),5)

        print("Contours: ",circle_count," circles detected")
    return img,contours

''' Circle detection using HoughTransform '''
def get_circles_hough(img):
    b = 0 #number of circles detected
    img = gray_conversion(img)
    # Transform picture to 3-channel greyscale for better contour visibility
    cimg = img.copy()
    cimg = cv2.cvtColor(cimg,cv2.COLOR_GRAY2BGR)

    img = cv2.medianBlur(img, 5)
    circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1 , 30, np.array([]), 10,15,1,20)
    if circles != None:
        a, b, c = circles.shape
        print("Hough: ",b," circles detected")
        for i in range(b):
            cv2.circle(cimg, (circles[0][i][0], circles[0][i][1]), circles[0][i][2], (0, 255, 255), 3, cv2.CV_AA)
            cv2.circle(cimg, (circles[0][i][0], circles[0][i][1]), 2, (255, 255, 0), 3, cv2.CV_AA) # draw center of circle

    return cimg, positions

''' DOESN'T WORK: Background Subtractor '''
def get_background(img):
    bg_operator = cv2.BackgroundSubtractorMOG()
    img_bg = bg_operator.apply(img)
    img_bg = cv2.imdecode(img_bg,cv2.IMREAD_GRAYSCALE)
    return img_bg

''' Red contours extraction '''
def extract_red(img):
    i=0
    max_area=0
    img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    # Returns all pixels that lie in specified range
    thresh = cv2.inRange(img_hsv,np.array((0,80,80)),np.array((20,255,255)))

    # Finds all  contours from binary image (wihtout hierarchy) Possibly RETR_EXTERNAL works too
    contours, hierarchy=cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    # Find contour with maximum area
    tmp=255*np.ones(img.shape,dtype=np.uint8)
    for cnt in contours:
        # Check if new best contour
        area = cv2.contourArea(cnt)
        if area > max_area:
            max_area = area
            best_cnt = cnt

        cv2.drawContours(img,[cnt],-1,(0,255/len(contours)*i,0),3)
        cv2.drawContours(tmp,[cnt],-1,(0,255/len(contours)*i,0),3)
        i+=1

    cx,cy = get_centroid(cnt)

    # Convert colors for correct display in Matplotlib:
    img = rgb_conversion(img)

    cv2.circle(img,(cx,cy),20,(0,255,255),2)
    return img,tmp,contours,(cx,cy)

''' Extract cirlces from different contours '''
def extract_circles(img,contours):
    cimg = gray_conversion(img)
    cimg = cv2.cvtColor(cimg,cv2.COLOR_GRAY2BGR)
    contours_circle = []

    # Define circle to be matched
    radius = 20
    thresh_fit = 0.10 # threshold underneath which shape is considered circle

    size =  int(radius*2+10)
    a = b =  int(round(size/2))
    y,x = np.ogrid[-a:size-a,-b:size-b]
    circ = x*x + y*y <= radius*radius
    mask = np.zeros((size,size),dtype=np.uint8)
    mask[circ] = 255
    mask_cpy=np.copy(mask)
    # ƒind contours (changes the image mask! )
    circle, h = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

    show_img = np.zeros(mask.shape,dtype=np.uint8)
    for cnt in circle:
        cv2.drawContours(show_img,[cnt],-1,(255,255,255),5)
    #plt.figure(),plt.imshow(show_img,'gray'),plt.show(block=False)

    i = 1
    max_fit = 100
    for cnt in contours:
        print("contour number:",i)
        i+=1

        # Count vertices
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        print("length:",len(cnt))
        print("number of vertices:",len(approx))
        np_circle = np.array(circle[0],dtype=np.int32)

        # Compare to test circle
        shape_fit = cv2.matchShapes(np_circle,cnt,cv.CV_CONTOURS_MATCH_I1,0)
        print("fit:",shape_fit)
        if shape_fit < max_fit:
            max_fit = shape_fit
            best_cnt = cnt

        if shape_fit < thresh_fit:
            contours_circle.append(cnt)

    cx,cy = get_centroid(best_cnt)

    for cnt in contours_circle:
        cv2.drawContours(cimg,cnt,-1,(0,255,255),2)

    plt.figure(2),plt.imshow(cimg),plt.show(block=False)
    return cimg, (cx,cy)

''' Apply geometric transformation to get "view from top"  '''
def manual_transformation(img):
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

''' ----------------   Main  --------------- '''
choice = "y"
while True:
    try:
        if choice == "y":
            # Close previous plots
            plt.close('all')

            # Read image
            inputfile=get_filename()
            if inputfile == '':
                img = get_image()
            else:
                img = cv2.imread(inputfile,cv2.IMREAD_COLOR)

            img_flat = img

            # Find robot position using different methods
            # Extract red shapes
            img_red,img_temp,cont_red,positions = extract_red(img_flat)
            # Extract image contours
            img_match, positions = extract_circles(img_temp,cont_red)
            img_hough, positions = get_circles_hough(img_temp)

            #imgs = [img_cont,img_hough,img_red]
            imgs = {'extract_red':img_red,'circles_match':img_match,'circles_hough':img_hough,
                    }
            n = len(imgs)
            plt.figure(figsize=(10,10))
            i = 1
            for (tit,im) in iteritems(imgs):
                plt.subplot(n,1,i),plt.imshow(im)
                plt.title(tit)
                i+=1

            plt.show(block=False)

        elif choice == "n":
            sys.exit(1)
        choice = raw_input("Do you want to perform another localisation? (y/n)")
    except (KeyboardInterrupt, SystemExit):
        print("Program terminated by user")
        sys.exit(1)
