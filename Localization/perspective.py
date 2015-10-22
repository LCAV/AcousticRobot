#-*- coding: utf-8 -*-
#!/usr/bin/env python
from __future__ import division
from __future__ import print_function
import cv2, sys, getopt, urllib, operator
import sys
import numpy as np
import cv2.cv as cv
import matplotlib.pyplot as plt
import marker_calibration as mark
import time
import get_image as get

DEBUG = 1
USAGE = '''
usage:
Option 1:
file.py -i <inputfile> [-o <outputpath>]

manipulations are done on inputfile (and saved in outputpath with
cameranumber in name)

Option 2:
file.py [-o <outputpath>]

manipulations are done on image from stream of indicated camera
(and saved in outputpath with camera number in name)
'''

'''--------------  Basics ------------------ '''
def gray_conversion(img):
    ''' Create 1-channel CV_8U image '''
    if img.dtype != 'uint8' or len(img.shape) != 2:
        img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    return img
def rgb_conversion(img):
    ''' Convert BGR (opencv) to RGB (matplotlib) '''
    b,g,r = cv2.split(img)
    return cv2.merge([r,g,b])
def get_centroid(cnt):
    ''' Get centroid from contour '''
    # Find centroid of contour
    M = cv2.moments(cnt)
    cy,cx = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
    return cx,cy
def visualization(imgs,n_cam,colorbar=0,switch=0):
    ''' Visualize results in figures named by their title.

    _Parameters_:


    '''
    n = len(imgs)
    for (tit,im) in imgs.items():
        #plt.subplot(n,1,i),plt.imshow(im)
        plt.close(tit+str(n_cam)),plt.figure(tit+str(n_cam))
        if switch == 0:
            plt.imshow(im)
        else:
            plt.imshow(im, origin = 'lower')
        if colorbar == 1:
                plt.colorbar()
        plt.title(tit+str(n_cam))
    plt.show(block=False)

'''----------  Program handling ------------ '''
class InteractivePlotN:
    ''' Plot where N reference points can be picked by mouse-click'''
    def __init__(self,pict,name,number):
        self.number = number
        self.pos = []
        self.pict = pict
        self.name = name
        self.hsv = cv2.cvtColor(self.name,cv2.COLOR_RGB2HSV)
        self.number = int(number)
    def setpoints(self):
        if self.number==1:
            print("Click on the robot head")
        else:
            print("Click on the {0} reference points in order".format(self.number))
        self.cid = self.pict.canvas.mpl_connect('button_press_event',self.onclick)
    def onclick(self,event):
        x = event.ydata
        y = event.xdata
        self.pos.append([x,y])
        n = len(self.pos)
        range_hsv = self.hsv[x][y]
        print('{0}: {1:5.2f},{2:5.2f}'.format(n,x,y))
        #print('HSV:',range_hsv)
        if n==self.number:
                plt.close()
    def draw(self):
        plt.imshow(self.name)
        plt.show()

def get_parameters():
    ''' Get filename from command line '''
    inputfile = ''
    outputpath = ''
    try:
        opts,args = getopt.getopt(sys.argv[1:],"i:o:",
                                  ["ifile=","opath="])
    except getopt.GetoptError:
        print(USAGE)
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
        elif opt in ("-o","--opath"):
            outputpath=arg

    return inputfile,outputpath
def write_ref(dirname,name,pts_img,M,pts_obj):
    ''' Save reference points positions in file, overwriting old results

    _Parameters_:
    dirname = directory
    name = name of file
    pts_img = np array with image positions of reference points.
    M = geometric transformation np matrix(3x3) obtained from geometric_transformationN
    pts_obj = np array with object positions of reference points

    format of pts_img, pts_obj: row i: (pi_x, pi_y) with i going from 1 to m
    (m = number of reference points)

    _Returns_: Nothing'''
    with open(dirname+name,"w") as f:# overwrite
        for pt in pts_img:
            f.write(str(pt[0])+"\t"+str(pt[1])+"\n")
        for m in M:
            f.write(str(m[0,0])+"\t"+str(m[0,1])+"\t"+str(m[0,2])+"\n")
        for pt in pts_obj:
            pt = pt*10 #points in mm
            f.write(str(pt[0])+"\t"+str(pt[1])+"\n")
def write_pos(dirname,name,p):
    ''' save robot position (img or obj) in file, appending to old results

    _Parameters_:
    dirname = directory
    name = name of file
    p = np array with robot position (x,y)

    _Returns_: Nothing'''
    with open(dirname+name,"a") as f: # append
        if p.any():
            for pt in p:
                f.write(str(pt)+'\t')
            f.write("\n")

def create_summary(img_flat,pts_obj,p=np.zeros(1)):
    img_summary = rgb_conversion(img_flat)
    # create summary image
    i = 0
    for pts in pts_obj:
        cv2.circle(img_summary,(int(pts[0]),int(pts[1])),3,
                    (255,0,0),1,cv2.CV_AA)
        i+=1
    if not p.all() == 0:
        p3D = np.matrix([p[0],p[1],1])
        test_p = M*p3D.T
        test_p = test_p/test_p[2]
        cv2.circle(img_summary,(int(test_p[0]),int(test_p[1])),5,
                    (255,255,0),1,cv2.CV_AA)
    return img_summary

def save_open_images(outputpath,n_cam,loopcounter=''):
    for i in plt.get_figlabels():
        plt.figure(i)
        plt.savefig(outputpath+str(loopcounter)+"_"+str(i)+'.png')

''' --------   Image Processing ------------ '''
def get_circles_count(img,contours,t,w,r):
    ''' Circles detection by counting pixels around contour centers '''
    t_pixels=r*4 #minimum distance between two circle centers

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
        col_inside = inside.cumsum()[-1]/inside.shape[0]
        if DEBUG:
            print("Count: Average color around circle:",col_around)
            print("Count: Average color inside circle:",col_inside)

        if col_around <= t and col_inside > t:
            #print("Count: Circle found: ",int(cx),int(cy))
            centers.append([cx,cy])

    # Remove duplicates
    centers.sort()
    rmv = []
    for i in range(len(centers)-1):
        diff = abs(np.subtract(centers[i],centers[i+1]))
        mean = np.mean([centers[i],centers[i+1]],axis = 0)
        if np.array([diff<t_pixels]).all():
            if DEBUG:
                print("Count: duplicate found: ",centers[i],centers[i+1])
            centers[i+1] = mean
            rmv.append(i)

    keep = np.delete(range(len(centers)),rmv)
    centers = np.take(centers,[keep],axis=0)
    centers=np.array(centers,dtype=np.uint)

    # Visualisation
    if len(centers[0])>0:
        print("Count: ", len(centers[0]), " circles detected: ",centers[0])
    cimg = img.copy()
    cimg = cv2.cvtColor(cimg,cv2.COLOR_GRAY2BGR)
    for i in range(len(centers[0])):
        cv2.circle(cimg,(centers[0][i][1],centers[0][i][0]),r,(0,255,0),2,cv2.CV_AA)
        cv2.circle(cimg,(centers[0][i][1],centers[0][i][0]),2,(0,0,0),2,cv2.CV_AA)
    return cimg, centers
def extract_color(img,range_min,range_max,r):
    ''' Color contours extraction '''
    i=0
    max_area=0
    min_area = r**2*np.pi/15
    print("Extract: minimum area = ", min_area)
    #min_area = 50
    best_cnt = 1
    cx = cy = 0

    img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    img = rgb_conversion(img)

    # Returns all pixels that lie in specified range
    if range_min[0] > range_max[0]:
        range_int = np.array([180,range_max[1],range_max[2]],dtype=np.uint8)
        img_diff1 = cv2.inRange(img_hsv,range_min,range_int)
        range_int = np.array([0,range_min[1],range_min[2]],dtype=np.uint8)
        img_diff2 = cv2.inRange(img_hsv,range_int,range_max)
        img_diff = cv2.add(img_diff1,img_diff2)
    else:
        img_diff = cv2.inRange(img_hsv,range_min,range_max)
    # Finds all  contours from binary image (wihtout hierarchy) Possibly RETR_EXTERNAL works too
    img_cont=img_diff.copy() #BECAUSE findContours changes input picture!
    contours, hierarchy=cv2.findContours(img_cont,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    contours_big = []
    # Find contour with maximum area
    for cnt in contours:
        # Check if new best contour
        area = cv2.contourArea(cnt)
        if DEBUG:
            print("Extract: area:",area)
        if area > max_area:
            #print("Extract: new best area: ",area)
            max_area = area
            best_cnt = cnt
        if area > min_area:
            contours_big.append(cnt)
            cv2.drawContours(img,[cnt],-1,(0,255,0),3)# Green
        i+=1
    try: # contours found
        cy,cx = get_centroid(best_cnt)
        cv2.circle(img,(cx,cy),20,(0,255,255),2)
    except: # no contours found
        cx,cy = 0
    return img,contours_big,(cx,cy),img_diff
def manual_calibration(img,n,R):
    ''' Determine shade of red '''
    #Get point of interest
    img=rgb_conversion(img)
    img_hsv = cv2.cvtColor(img,cv2.COLOR_RGB2HSV)

    img_i = InteractivePlotN(plt.figure(),img,n)
    img_i.setpoints()
    img_i.draw()
    points = img_i.pos

    img_mask = np.zeros(img.shape)

    # Create mask around contour center
    x,y = np.ogrid[:img.shape[0],:img.shape[1]]
    for i in range(n):
        if n >= 1:
            cy = points[i][1]
            cx = points[i][0]
        circle = (x-cx)*(x-cx) + (y-cy)*(y-cy) <= R*R
        circle = circle.astype(np.uint8)
        img_mask[circle==1]=1

    return img_mask, points
def automatic_calibration(img,range_min,range_max,thresh_diff,col_diff):
    ''' Determine shade of red automatically '''

    img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    # Spectrum contains limit (180)
    if range_min[0] > range_max[0]:
        range_int = np.array([180,range_max[1],range_max[2]],dtype=np.uint8)
        img_diff1 = cv2.inRange(img_hsv,range_min,range_int)
        range_int = np.array([0,range_min[1],range_min[2]],dtype=np.uint8)
        img_diff2 = cv2.inRange(img_hsv,range_int,range_max)
        img_diff = cv2.add(img_diff1,img_diff2)
    else:
        img_diff = cv2.inRange(img_hsv,range_min,range_max)
    # get corresponding colors
    colors = img_hsv[img_diff>0]

    # confidence interval
    range_mean = np.mean(colors,axis=0)
    range_dev = np.std(colors,axis=0,ddof=1)
    # evaluate weather all elements of each row are in given confidence interval
    colors_clean = colors[np.all(abs(colors-range_mean)/range_dev<=col_diff,axis=1)]

    # find min and max of each color components
    rmin = np.amin(colors_clean,axis=0)
    rmax = np.amax(colors_clean,axis=0)
    return rmin,rmax,colors_clean
def get_histograms(img,img_mask,n):
    ''' Determine two shades of red automatically '''
    img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    img_mask = img_mask.astype(np.uint8)
    hsv_reduced = img_mask*img_hsv
    h,s,v=cv2.split(hsv_reduced)
    h[h==0] = 179
    s[s==0] = 255
    # 1D Histogram
    plcolors = ('r','g')
    plt.figure(n)
    #h = cv2.calcHist(img_hsv,[0],img_mask,[180],[0,180])
    hist_h = np.zeros(256)
    hist_h[0:180],bins_h = np.histogram(h,180,[0,178])
    plt.plot(hist_h,color=plcolors[0])
    plt.legend('h')
    #hist_s = cv2.calcHist(img_hsv,[1],img_mask,[256],[0,256])
    hist_s, bins_s = np.histogram(s,256,[0,254])
    plt.plot(hist_s,color=plcolors[1])
    plt.legend('s')
    plt.show(block=False)
    return hist_h,hist_s

def imagepoints_auto(img,r1,n,t,col_min,col_max,points,r2):
    ''' Get positions of robot automatically (ignoring reference poitns)'''
    x,y = np.ogrid[:img.shape[0],:img.shape[1]]
    img_reduced=img.copy()
    r2 = r2*5
    for c in points:
        # Create mask around contour center
        cx = c[0,1]
        cy = c[0,0]
        circle = (x-cx)*(x-cx) + (y-cy)*(y-cy) <= r2*r2
        circle = circle.astype(np.uint8)
        img_reduced[circle==1] = [255,255,255]
    return imagepoints(img_reduced,r1,n,t,col_min,col_max,1)

def imagepoints(img,r,n,t,col_min,col_max,reduced=0):
    ''' Get positions of reference points or robot. '''
    img_reduced = img.copy()
    w = r*5
    col_diff = 2
    counter = 1
    colors= np.zeros((2550,200,3))
    colors = colors.astype(np.uint8)
    points=0
    if not reduced:
        # Get regions of interest
        img_mask,points = manual_calibration(img,n,r*5)
        img_reduced[img_mask==0] = 255
        img_reduced = img_reduced.astype(np.uint8)

    # refine color range
    col_min,col_max,col_clean = automatic_calibration(img_reduced,col_min,
                                                      col_max,t,col_diff)
    #hist_h, hist_s = get_histograms(img,img_mask,n*10)

    # color
    img_color,cont_color,pos,th = extract_color(img_reduced,col_min,col_max,r)

    # circles
    circ_color,pos_color = get_circles_count(th,cont_color,
                                            t,w,r)
    if pos_color.shape[1] == n:
        print("Image points found")
    else:
        print("Did not find image points")

    colors[10*(counter-1):10*counter,0:100,:]=col_min
    colors[10*(counter-1):10*counter,101:200,:]=col_max

    col = colors[0:10*counter,:,:]#Keep only non-zero colors
    col = cv2.cvtColor(col,cv2.COLOR_HSV2RGB)

    #plt.close(1),plt.figure(1),plt.imshow(col)
    #plt.show(block=False)
    #test_pause = raw_input("pause, enter to continue")

    pos_color = restore_order(points,pos_color,img.shape[1]/10)

    # Order of found points has to be reversed.
    if n == 1:
        px = pos_color[0][0][1]
        py = pos_color[0][0][0]
        pos_color = np.array([px,py])
    else:
        pos_color = np.vstack(([pos_color[:,1],pos_color[:,0]])).T

    return img_color,circ_color,pos_color,th

''' --------------  Geometry ---------------'''
def objectpoints(m,name):
    ''' reads objectpoint positions from file.
    _Parameters_:
    m = number of points
    name = file name

    _Returns_:
    pts_obj = object posotions of reference points (in cm)
    M1 = object returned by MarkerSet.
'''
    # Initialisation
    dim = 2
    marker_diameter = 0.040 # in m
    # Read relative positions (in m)
    D = np.genfromtxt(name,delimiter=";",skiprows=1,
                   skip_header=1,filling_values=0)
    # Delete first column
    D = D[:,1:]
    # Fill other half of the matrix:
    D = D + D.T
    # Add marker diameter to all values that are not 0
    D = D+marker_diameter
    D[D==marker_diameter]=0
    # Get postions
    M1 = mark.MarkerSet(m=m,dim=dim,diameter=marker_diameter)
    M1.fromEDM(D**2)
    M1.normalize()
    #left and lower margin respectively, in m
    pts_obj = M1.X.T*100 # pts in cm.
    return pts_obj,M1
def geometric_transformationN(img,pts_obj,pts_img,size):
    ''' Find Homography for N points '''
    pts_obj = pts_obj.astype(np.float32)
    pts_img = pts_img.astype(np.float32)
    M,__ = cv2.findHomography(pts_img,pts_obj)
    img_flat = cv2.warpPerspective(img,M,size)
    return img_flat, np.matrix(M)
def create_order(pts):
    ''' reorganizes points in order top to bottom, left to right '''
    pts_3D_top=pts[0][:2]
    pts_3D_bot=pts[0][-2:]
    pt1,pt2=sorted(pts_3D_top,key=operator.itemgetter(1))
    pt3,pt4=sorted(pts_3D_bot,key=operator.itemgetter(1))
    pts = [pt1,pt2,pt3,pt4]
    pts = np.array(pts)
    return np.array(pts)
def restore_order(original,moved,min_dist):
    ''' reorganize points in order of clicking '''
    if moved.shape[1] > 1:
        i = 0
        original = np.array(original)
        pts = np.zeros(original.shape,dtype=np.float32)
        for pos in original:
            comp = abs(moved-pos)<min_dist
            maxim = np.where(comp[0][:,0]&comp[0][:,1])[0]
            #print(i,": ",comp,maxim)
            pts[i,:] = moved[0][maxim[0],:]
            i+=1
    else:
        pts = moved
    return pts
def format_points(pts_obj,margin,mask = 'all'):
    # move points to positive range
    pts_obj = pts_obj - np.amin(pts_obj,axis=0)

    # stretch image
    pts_obj = pts_obj.astype(np.float32)
    pts_obj = pts_obj + margin
    size = np.amax(pts_obj,axis=0)+margin

    # choose only certain points if specified
    if mask != 'all':
        mask = mask*np.ones((2,len(mask)))
        pts_obj=pts_obj[mask.T==1]
        pts_obj = pts_obj.reshape((len(pts_obj)/2,2))

    img_test = np.zeros((size[1],size[0]))
    img_test = img_test.astype(np.uint8)
    for pt in pts_obj:
        cv2.circle(img_test,(int(pt[0]),int(pt[1])),10,(255),3,cv2.CV_AA)

    return img_test,pts_obj,(int(size[0]),int(size[1]))


''' ----------------   Main  --------------- '''
if __name__ == "__main__":
    choice = "y"
    #DEBUG = 1
    while True:
        try:
            if choice == "y":
                plt.close('all')
                img = ''
                n_cam = 'None'
                n_pts = 4 #number of reference points
                mask = [0,0,1,1,1,1] #choice of reference points
                while True:
                    try:
                        n_cam = int(n_cam)
                        break
                    except:
                        n_cam = raw_input("Please enter camera number: ")
                #---------------- Get parameters --------------#
                inputfile,outputpath = get_parameters()
                if inputfile == '':
                    img = get.get_image(n_cam)
                else:
                    img = cv2.imread(inputfile,cv2.IMREAD_COLOR)
                if outputpath != '':
                    if outputpath[-1]!="/":
                        outputpath = outputpath+"/"
                    outputfile = outputpath+"img"+str(n_cam)+".jpg"
                    cv2.imwrite(outputfile,img)

                #----------- Extract reference points----------#
                # find orange
                col_min = np.array([175,100,0],dtype=np.uint8)
                col_max = np.array([20,250,255],dtype=np.uint8)
                r=30
                t=100
                img_org,circ_org,pts_img,th_org = imagepoints(img,r,n_pts,t,
                                                              col_min,col_max)
                if pts_img.shape[1] == 0:
                    print("No reference points found. Try again!")
                    break

                #-------------- Find robot position  ----------#
                choice = "n"
                choice = raw_input("Find robot position?(y/n)")
                if choice == "y":
                    col_min = np.array([150,100,0],dtype=np.uint8)
                    col_max = np.array([20,250,255],dtype=np.uint8)
                    r = 40
                    t = 50
                    img_red,circ_red,p,th_red = imagepoints(img,r,1,t,
                                                            col_min,col_max)

                else:
                    p = np.array([np.zeros((1,2))])
                    img_red = np.zeros(img.shape)
                    th_red = np.zeros(img.shape)
                    circ_red = np.zeros(img.shape)
                #-------------- Project image to 2D -----------#
                # Get real positions
                # only for testing
                margin = np.array([40,46]) #in cm
                pts_obj2,M = objectpoints(m,'input/objectpoints.csv')
                # position of real points in cm, pt = (x,y)
                pts_obj_test=np.array([[0,0],[70,0],[70,70],[0,70]],dtype=np.float32)
                pts_obj_test=np.vstack((pts_obj_test,[50,33],[16,90]))
                pts_obj_test = pts_obj_test+margin
                # Ony choose certain object points

                img_test,pts_obj,size = format_points(pts_obj2,margin,mask)
                img_flat,M = geometric_transformationN(img,pts_obj,pts_img,size)

                ##------------------ Summary ------------------#
                img_summary = create_summary(img_flat,pts_obj,p)

                # write results into file
                if outputpath != '':
                    current_time = str(int(time.mktime(time.gmtime())))
                    name='ref_' +str(n_cam)+'_'+current_time+str('.txt')
                    write_ref(outputpath,name,pts_img,M,pts_obj)
                    name='pos_img'+str(n_cam)+'.txt'
                    write_pos(outputpath,name,p)
                #---------------  Visualization    ------------#
                h,s,v = cv2.split(cv2.cvtColor(img,cv2.COLOR_BGR2HSV))
                imgs = {'img_h':h,
                        'img_s':s}
                visualization(imgs,n_cam,1)
                imgs = {'img_org':img_org,
                        'circ_org':circ_org}
                visualization(imgs,n_cam,)
                imgs = {'img_red':img_red,
                        'circ_red':circ_red}
                visualization(imgs,n_cam)
                imgs = {'summary':img_summary}
                visualization(imgs,n_cam,0,1)
                if outputpath != '':
                    save_open_images(outputpath,n_cam)
            elif choice == "n":
                sys.exit(1)
            choice = raw_input("Do you want to perform another localisation? (y/n)")
        except (KeyboardInterrupt, SystemExit):
            print("Program terminated by user")
            sys.exit(1)
