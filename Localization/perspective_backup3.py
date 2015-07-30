# -*- coding: utf-8 -*-
from __future__ import division
import cv2, sys, getopt, urllib, operator
import numpy as np
import cv2.cv as cv
import matplotlib.pyplot as plt

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
''' Visualize results '''
def visualization(imgs,nexti):
    n = len(imgs)
    i = nexti
    for (tit,im) in imgs.items():
        #plt.subplot(n,1,i),plt.imshow(im)
        plt.close(i),plt.figure(i),plt.imshow(im)
        plt.title(tit)
        i+=1
    plt.show(block=False)
    return i

'''----------  Program handling ------------ '''

''' Plot where 4 indicator points can be picked by mouse-click'''
class InteractivePlot:
    def __init__(self,pict,name,number):
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
        self.number = int(number)
    def setpoints(self):
        if self.number==4:
            print("Choose the 4 indicator points (left to right, top to bottom)")
        else:
            print("Choose the point that contains the color of interest")

        self.cid = self.pict.canvas.mpl_connect('button_press_event',self.onclick)
    def onclick(self,event):
        if self.x1==0.0 and self.y1 == 0.0:
            self.x1 = event.xdata
            self.y1 = event.ydata
            print('1: {0:5.2f},{1:5.2f}'.format(self.x1,self.y1))
            if self.number!=4:
                plt.close()
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
        if self.number == 4:
            return self.x1,self.y1,self.x2,self.y2,self.x3,self.y3,self.x4,self.y4
        else:
            return self.x1,self.y1
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
            if cv2.imread(arg,0).any():
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
''' Get edges using Canny '''
def get_edges_canny(img,param):
    thresh1 = param
    thresh2 = param/2

    edges = cv2.Canny(img,thresh1,thresh2)
    plt.figure(5),plt.imshow(edges,'gray'),plt.title('Canny'),plt.show(block=False)
    return edges
''' Circle detection using HoughTransform '''
def get_circles_hough(img):
    b = 0 #number of circles detected
    p1,p2,rmin,rmax = (100,30,0,0)


    img = gray_conversion(img)
    # Transform picture to 3-channel greyscale for better contour visibility
    cimg = img.copy()
    cimg = cv2.cvtColor(cimg,cv2.COLOR_GRAY2BGR)

    # ƒor debugging only
    #edg = get_edges_canny(img,p1)
    circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1 , 10, np.array([]),
                               p1,p2,rmin,rmax)
    if circles != None:
        a, b, c = circles.shape
        print("Hough: ",b," circles detected: ",circles)
        for i in range(b):
            cv2.circle(cimg, (circles[0][i][0], circles[0][i][1]), circles[0][i][2], (0, 255, 0), 3, cv2.CV_AA)
            cv2.circle(cimg, (circles[0][i][0], circles[0][i][1]), 2, (0, 255, 0), 3, cv2.CV_AA) # draw center of circle
    else:
        print("Hough: no circles detected")

    return cimg, circles
''' Circles detection by counting pixels around contour centers '''
def get_circles_count(img,contours,t_around,t_inside,w,r):
    t_pixels=10 #minimum distance between two circle centers

    img = gray_conversion(img)
    x,y = np.ogrid[:img.shape[0],:img.shape[1]]
    centers=[]
    for cont in contours:
        # Create mask around contour center
        c = np.mean(cont,axis=0)
        cy = c[0][0]
        cx = c[0][1]
        square = (abs(x-cx)<w)  & (abs(y-cy)<w)
        square = square.astype(np.uint8)
        circle = (x-cx)*(x-cx) + (y-cy)*(y-cy) <= r*r
        circle = circle.astype(np.uint8)
        circle[circle==1]=2
        circle[circle==0]=1
        mask = circle*square
        # Count pixels
        around = img[mask==1]
        inside = img[mask==2]
        col_around = around.cumsum()[-1]/around.shape[0] #average color of aroud
        print("average color around circle:",col_around)
        col_inside = inside.cumsum()[-1]/inside.shape[0]
        print("average color inside circle:",col_inside)

        if  col_around > t_around and col_inside < t_inside:
            #print("circle detected")
            centers.append([cx,cy])

    # Remove duplicates
    centers.sort()
    rmv = []
    for i in range(len(centers)-1):
        diff = abs(np.subtract(centers[i],centers[i+1]))
        mean = np.mean([centers[i],centers[i+1]],axis = 0)
        if np.array([diff<t_pixels]).all():
            print("Count: duplicate found: ",centers[i],centers[i+1])
            centers[i+1] = mean
            rmv.append(i)

    keep = np.delete(range(len(centers)),rmv)
    centers = np.take(centers,[keep],axis=0)
    centers=np.array(centers,dtype=np.uint)

    # Visualisation
    print("Count: ", len(centers[0]), " circles detected: ",centers[0])
    cimg = img.copy()
    cimg = cv2.cvtColor(cimg,cv2.COLOR_GRAY2BGR)
    for i in range(len(centers[0])):
        cv2.circle(cimg,(centers[0][i][1],centers[0][i][0]),20,(0,255,0),1,cv2.CV_AA)
        cv2.circle(cimg,(centers[0][i][1],centers[0][i][0]),2,(0,0,0),1,cv2.CV_AA)
    return cimg, centers
''' Circle detection by matchShape '''
def get_circles_match(img,contours):
    circle_contours = []
    circle_centers = []
    best_cnt=0
    radius = 5
    thresh_fit = 0.10 # threshold underneath which shape is considered circle
    max_fit = 100

    # Define circle to be matched
    size =  int(radius*2+10)
    a = b =  int(round(size/2))
    y,x = np.ogrid[-a:size-a,-b:size-b]
    circ = x*x + y*y <= radius*radius
    mask = np.zeros((size,size),dtype=np.uint8)
    mask[circ] = 255
    mask_cpy=np.copy(mask)

    cimg = gray_conversion(img)
    cimg = cv2.cvtColor(cimg,cv2.COLOR_GRAY2BGR)

    # ƒind contours (changes the image mask! )
    circle, h = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

    show_img = np.zeros(mask.shape,dtype=np.uint8)
    for cnt in circle:
        cv2.drawContours(show_img,[cnt],-1,(255,255,255),5)
    #plt.figure(),plt.imshow(show_img,'gray'),plt.show(block=False)

    i = 1
    for cnt in contours:
        #print("contour number:",i)
        i+=1

        # Count vertices
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        #print("length:",len(cnt))
        #print("number of vertices:",len(approx))

        # Compare to test circle
        np_circle = np.array(circle[0],dtype=np.int32)
        shape_fit = cv2.matchShapes(np_circle,cnt,cv.CV_CONTOURS_MATCH_I1,0)
        #print("fit:",shape_fit)

        # ƒind best fit (currently not used)
        if shape_fit < max_fit and shape_fit != 0.0:
            max_fit = shape_fit
            best_cnt = cnt
        # find circles
        if shape_fit < thresh_fit and shape_fit != 0.0:
            circle_contours.append(cnt)
            cv2.drawContours(cimg,cnt,-1,(0,255,255),1)
            cx,cy = get_centroid(cnt)
            circle_centers.append([cx,cy])

    print("Match: ",len(circle_centers), " circles detected: ",circle_centers)
    return cimg, circle_centers
''' Color contours extraction '''
def extract_color(img,range_min,range_max):
    i=0
    max_area=0
    cx = cy = 0

    img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    img = rgb_conversion(img)

    #plt.imshow(img),plt.show(block=False)


    # Returns all pixels that lie in specified range
    thresh = cv2.inRange(img_hsv,range_min,range_max)

    # Finds all  contours from binary image (wihtout hierarchy) Possibly RETR_EXTERNAL works too
    contours, hierarchy=cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    # Find contour with maximum area
    tmp=255*np.ones(img.shape,dtype=np.uint8)
    for cnt in contours:
        # Check if new best contour
        area = cv2.contourArea(cnt)
        if area > max_area:
            #print("new area: ",area)
            max_area = area
            best_cnt = cnt

        cv2.drawContours(img,[cnt],-1,(0,255,0),3)# Green
        cv2.drawContours(tmp,[cnt],-1,(0,255,0),3)# Green
        i+=1

    # Convert colors for correct display in Matplotlib:
    try:
        cx,cy = get_centroid(best_cnt)
        cv2.circle(img,(cx,cy),20,(0,255,255),2)
        print("red contours found")
    except:
        print("No red contours found")

    return img,tmp,contours,(cx,cy),thresh
''' Determine shade of red '''
def manual_calibration(img):
    #Get point of interest
    img=rgb_conversion(img)
    img_i = InteractivePlot(plt.figure(),img,1)
    img_i.setpoints()
    img_i.draw()
    x,y = img_i.getpoints()
    print("RGB: ",img[y][x])
    img = cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
    range_hsv = img[y][x]
    print("HSV: ",range_hsv)

    # Works well for orange:
    # range_hsv=np.array([ 19, 236, 240])
    range_hsv=range_hsv.astype(np.int32)
    range_min = range_hsv-20
    range_min[range_min<0]=0
    range_max = range_hsv+20
    range_max[range_max>255]=255
    if range_max[0]>180:
        range_max[0] = 180

    range_min=range_min.astype(np.uint8)
    range_max=range_max.astype(np.uint8)

    return range_hsv, range_min, range_max
''' Determine shade of red automatically '''
def automatic_calibration(img,thresh_diff,col_diff):
    img_ref = cv2.imread("pics/real.jpg",cv2.IMREAD_COLOR)
    img_ref = cv2.GaussianBlur(img_ref,(3,3),0)
    img = cv2.GaussianBlur(img,(3,3),0)

    # get difference mask
    img_diff = cv2.subtract(img, img_ref)
    img_diff = gray_conversion(img_diff)
    mask = np.zeros(img_diff.shape,dtype=np.uint8)
    mask[img_diff >= thresh_diff] = 1

    # get corresponding colors
    img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    colors = img_hsv[mask==1]

    # confidence interval
    range_mean = np.mean(colors,axis=0)
    range_dev = np.std(colors,axis=0,ddof=1)
    # evaluate weather all elements of each row are in given confidence interval
    colors_clean = colors[np.all(abs(colors-range_mean)/range_dev<=col_diff,axis=1)]

    # find min and max of each color components
    rmin = np.amin(colors_clean,axis=0)
    rmax = np.amax(colors_clean,axis=0)
    return rmin,rmax
''' Determine two shades of red automatically '''
def autoamtic_calibration2(img):
    img_ref = cv2.imread("pics/real.jpg",cv2.IMREAD_COLOR)
    img_ref = cv2.GaussianBlur(img_ref,(3,3),0)
    img = cv2.GaussianBlur(img,(3,3),0)

    img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    img_ref_hsv = cv2.cvtColor(img_ref,cv2.COLOR_BGR2HSV)

    # 1D Histogram
    hist = cv2.calcHist(img_hsv,[0],None,[180],[0,180])
    hist_ref = cv2.calcHist(img_ref_hsv,[0],None,[180],[0,180])
    hist_diff = abs(hist - hist_ref)
    hist_avg = np.divide(hist_diff,hist)
    hist_avg[~np.isfinite(hist_avg)]=0

    hists=[hist,hist_ref,hist_diff]
    plcolors = ('r','g','b')
    i = 0
    for h in  hists:
        plt.plot(h,color=plcolors[i])
        i+=1
    plt.show(block=False)
    i = 0

    plt.figure(2), plt.plot(hist_avg),plt.show(block=False)

    # 2D Histogram
    hist2 = cv2.calcHist(img_hsv,[0,1],None,[180,256],[0,180,0,256])
    hist_ref2 = cv2.calcHist(img_ref_hsv,[0,1],None,[180,256],
                                [0,180,0,256])
    hist_diff2 = abs(hist2-hist_ref2)
    sys.exit(1)

    # 3D Histogram
    hist3 = cv2.calcHist(img_hsv,[0,1,2],None,[180,256,256],
                        [0,180,0,256,0,256])
    hist_ref3 = cv2.calcHist(img_ref_hsv,[0,1,2],None,[180,256,256],
                            [0,180,0,256,0,256])
    hist_diff3 = abs(hist2 - hist_ref2)
    hists2=[hist3,hist_ref3,hist_diff3]
    plt.figure(3)
    for h in hists2:
        plt.subplot(131),plt.xlim([0,56])
        plt.plot(h[1,1,:],color=plcolors[i])
        plt.subplot(132),plt.xlim([0,56])
        plt.plot(h[1,:,1],color=plcolors[i],linestyle='_')
        plt.subplot(133),plt.xlim([0,80])
        plt.plot(h[:,1,1],color=plcolors[i],linestyle=':')
        i += 1

    plt.show(block=False)
    sys.exit(1)
    return 0

''' --------------  Geometry ---------------'''
''' Apply geometric transformation to get "view from top"  '''
def geometric_transformation(img,pts_2D,size,pts_3D=[]):
    if pts_3D == []:
        pts_3D = get_3D_points(img)
    M = cv2.getPerspectiveTransform(pts_3D,pts_2D)
    img_flat = cv2.warpPerspective(img,M,size)
    return img_flat
def get_3D_points(img):
    # Get indicator points
    img_i = InteractivePlot(plt.figure(),img,4)
    img_i.setpoints()
    img_i.draw()
    x1,y1,x2,y2,x3,y3,x4,y4 = img_i.getpoints()
    return  np.float32([x1,y1],[x2,y2],[x3,y3],[x4,y4])
def order_points(pts):
    pts_3D_top=pts[0][:2]
    pts_3D_bot=pts[0][-2:]
    pt1,pt2=sorted(pts_3D_top,key=operator.itemgetter(1))
    pt3,pt4=sorted(pts_3D_bot,key=operator.itemgetter(1))
    pts = [pt1,pt2,pt3,pt4]
    pts = np.array(pts)
    pts = [pts[:,1],pts[:,0]]
    return np.array(pts).T


''' ----------------   Main  --------------- '''
choice = "y"
while True:
    try:
        if choice == "y":
            plt.close('all')
            nexti = 1

            # Read image
            inputfile=get_filename()
            if inputfile == '':
                img = get_image()
            else:
                img = cv2.imread(inputfile,cv2.IMREAD_COLOR)

            #----------- Extract reference points----------#
            # range_min,range_max = manual_calibration(img)
            orange_min,orange_max = automatic_calibration(img,50,1)

            # orange
            img_orange,img_temp,cont_orange,pos,th = extract_color(img,orange_min,
                                                            orange_max)
            # circles
            #img_temp=cv2.GaussianBlur(img_temp,(5,5),0)
            #img_match, pos_match = get_circles_match(img_temp,cont_orange)
            img_count, pos_count = get_circles_count(img_temp,cont_orange,
                                                     250,200,20,5)
            #img_hough, pos_hough = get_circles_hough(img_temp)

            imgs = {'extract_color':img_orange,
                    'circles_count':img_count}
            nexti=visualization(imgs,nexti)

            #-------------- Project image to 2D -----------#
            width = 1800
            height = 2000
            r_width = 800
            r_height = 400
            r_margin = (width-r_width)/2
            pts_2D = np.float32([[r_margin,height-r_height],
                                 [width-r_margin,height-r_height],
                                 [r_margin,height],[width-r_margin,height]])
            pts_3D = pos_count.astype(np.float32)
            pts_3D = order_points(pts_3D)
            img_flat = geometric_transformation(img,pts_2D,(width,height),pts_3D)

            imgs = {'original image':rgb_conversion(img),
                    'projected image':rgb_conversion(img_flat)}
            nexti=visualization(imgs,nexti)
            #--------------Extract robot location----------#
            # red_hsv,red_min, red_max = manual_calibration(img_flat)
            # works well:
            red_min = np.array([156,190,235],dtype=np.uint8)
            red_max = np.array([180,230,255],dtype=np.uint8)
            img_red,img_tempred,cont_red,pos,th = extract_color(img_flat,red_min,red_max)
            # works with img:250,230,50,15
            # works with img_flat:254,251,200,50
            img_robot,pos_robot = get_circles_count(img_tempred,cont_red,
                                                    250,250,300,50)
            if pos_robot == []:
                pos_robot = pos[0][0]
            else:
                pos_robot = pos_robot[0][0]

            imgs = {'extract red':img_red,
                    'circles_count':img_robot}
            nexti=visualization(imgs,nexti)

        elif choice == "n":
            sys.exit(1)
        #choice = raw_input("Do you want to perform another localisation? (y/n)")
        choice = "n"
    except (KeyboardInterrupt, SystemExit):
        print("Program terminated by user")
        sys.exit(1)
