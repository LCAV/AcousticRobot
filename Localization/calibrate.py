#-*- coding: utf-8 -*-
#!/usr/bin/env python
##@package calibrate
# calibrate
#=============
# Contains Image and Camera classes.
#
#Contains all functions necessary for intrinsic and extrinsic camera calibration,
#including the triangulation between cameras for 3D reconstruction.
#
# All functions using more than one camera are not owned by an object.
#
# created by Frederike Duembgen, August 2015
#

from __future__ import division
from __future__ import print_function
from future.utils import iteritems
import numpy as np
import cv2
import cv2.cv as cv
import matplotlib.pyplot as plt
import csv
import get_image as get
import os
import sys
from scipy.linalg import svd
import itertools
import perspective as persp
import time

USAGE = '''
USAGE: calib.py [-o <output path>] [-c <camera path>]
[-n <camera_number>] [-f <fisheye on (0/1)>]
'''

def combinations(array,number):
    ''' returns vector with  all possible camera combinations
    of specified length (in vector) '''
    arr = np.matrix(array[:number])
    for val in itertools.product(array, repeat=number):
        val = np.array(val)
        # check if combination has no duplicates in itself
        if len(val)==len(set(val)):
            val.sort()
            # check if this is not yet in the vectors
            if arr.shape[0]==1 and not np.all(arr==val):
                arr = np.vstack((arr,val))
            elif not np.all(arr==val,axis=1).any():
                arr = np.vstack((arr,val))
    return arr
def get_param():
    ''' returns parameters from command line '''
    global USAGE
    out_dir = ''
    in_dir = ''
    n = 141
    fisheye = '0000'
    try:
        opts,args = getopt.getopt(sys.argv[1:],"o:c:n:f:",
                                  ['save=', 'cam=','n=','fisheye='])
    except getopt.GetoptError:
        print(USAGE)
        sys.exit(2)
    for opt, arg in opts:
        if opt in ('--save',"-o"):
            out_dir = str(arg)
        if opt in ("--cam", "-c"):
            in_dir = str(arg)
        if opt in ('--n',"-n"):
            n = int(arg)
        if opt in ('--fisheye',"-f"):
            fish = np.array(arg.split('0'))
            fisheye = [c=='1' for c in fish]
    if out_dir[-1]!="/":
        out_dir = out_dir+"/"
    if in_dir[-1]!="/":
        in_dir = in_dir + "/"
    return out_dir,in_dir,n,fisheye
def get_leastsquares(cam,pt,method='hz',r_height='',p_real=''):
    '''
    Solve least squares problem for point position with or without fixed height

    _Parameters_:

    cam:    nparray of Camera objects used for triangulation

    pt:     nparray of points to be measured

    method: 'hz' (default) uses Hartley Zisserman Aglorithm, 'my' uses own
            algorithm. Only use 'hz' only!

    r_height:   real height of robot in mm. if specified, the fixed height
                algorithm is used.

    p_real:     nparray of real position of robot (3x1), for error calculation
                only. default '' returns err2=err3=0.

    _Returns_:

    x: nparray of real positions of points. Size 3xN_pts

    err2: nparray of 2D errors. Size 1xN_pts

    err3: nparray of 3D errors. Size 1xN_pts
    '''
    A_part = dict()
    b_part = dict()
    x=err2=err3=0
    if method == 'hz':
        '''
        DLT Method of Hartley Zisserman, chapter 12.2
        if height specified: least squares algorithm can be applied.
        if not: use singular value decomposition to find vector corresponding
        to smallest singular value.
        '''

        # construct A as in p.312
        for i,c in enumerate(cam):
            f1 = c.Proj[:1]
            f2 = c.Proj[1:2]
            f3 = c.Proj[2:3]
            p = pt[i]
            x = p[0,0]
            y = p[0,1]
            A_part[i] = np.vstack((x*f3-f1,y*f3-f2))

        # Stack A matrices of each camera together.
        A_aug = np.vstack(A_part.values())

        # r_height specified:
        if type(r_height)!=str:
            a1 = A_aug.T[:1]
            a2 = A_aug.T[1:2]
            a3 = A_aug.T[2:3]
            a4 = A_aug.T[3:4]
            A_aug2 = np.vstack((a1,a2)).T
            b_aug = -a3*r_height-a4
            x_hat = (A_aug2.T*A_aug2).I*A_aug2.T*b_aug.T
            control = A_aug*np.vstack((x_hat,r_height,1))
            x =  np.vstack((x_hat,r_height))#control,0
        # r_height not specified:
        else:
            u,w,v = svd(A_aug)
            x_hat = v[-1]
            x_hat = x_hat/x_hat[3]
            x=x_hat[:3]
    else:
        ''' Own Method using algebraic transformations
        Not quite right because scaling factor not taken into account!
        _part[i] = np.vstack((x*f3-f1,y*f3-f2))

        '''
        if type(r_height)!=str:
            n = len(pt)
            U = np.zeros((3*n,n))
            for i,p in enumerate(pt):
                c = cam[i]
                U[3*i:3*i+3,i] = p
                (r1,r2,r3)=c.R.T[:,:]
                A_part[i]=np.hstack(([-c.C*r1.T,-c.C*r2.T]))
                b_part[i]=c.C*(r3.T*r_height+c.t)
            A_aug = np.hstack((U,np.vstack(A_part.values())))
            b_aug = np.vstack((b_part.values()))
            x_hat = (A_aug.T*A_aug).I*A_aug.T*b_aug
            x=np.vstack((x_hat[-2:,0],r_height))#x_hat[0:n,0]
        else:
            for i,p in enumerate(pt):
                A_part[i]=cam[i].Proj
                b_part[i]=p.T
            A_aug = np.vstack(A_part.values())
            b_aug = np.vstack(b_part.values())
            b_aug = b_aug.reshape((6,1))
            x_hat = (A_aug.T*A_aug).I*A_aug.T*b_aug
            x_hat = x_hat/x_hat[3]
            x=x_hat[:3]
    if type(p_real)!=str:
        x = x.reshape((3,1))
        p_real = p_real.reshape((3,1))
        err3 = np.sqrt(np.sum(np.power(p_real-x,2)))
        err2 = np.sqrt(np.sum(np.power(p_real[:2]-x[:2],2)))
    return x,err2,err3
def get_leastsquares_combinations(n_cameras,numbers,cam,pt,method='hz',r_height='',p_real=''):
    '''
    Applies get_leastsquares function to all possible combinations of length
    "number" made of elements of "n_cameras".

    _Parameters_:

    n_cameras: array of cameras to be tested (e.g. [139,141,145])

    numbers: array of numbers of cameras to be tested.

    cam: list of corresponding camera instances

    pt: vector of augmented image positions of point to be triangulated
    (matrix with [x,y,1]) in all cameras

    method: method to be used in leastsquares (with or without fixed height),
    see get_leastsquares() for more details.

    r_height: real height of point (if known)

    p_real: real position of point (if known)

    _Returns_:

    pts: 3D position

    2D: 2D error

    3D: 3D error

    arrs: list of camera number combination
    '''
    arrs = dict()
    errs2 = dict()
    errs3 = dict()
    pts = dict()
    j = 0
    for number in numbers:
        array=combinations(n_cameras,number)
        for arr in array:
            # extract data corresponding to camera array
            cams_selection=[]
            pts_selection=[]
            for i,c in enumerate(n_cameras):
                if c in arr:
                    cams_selection.append(cam[i])
                    pts_selection.append(pt[i])

            # calculate error
            p,err2,err3 = get_leastsquares(cams_selection,pts_selection,method,r_height,p_real)
            arrs[j] = arr
            errs2[j] = err2
            errs3[j] = err3
            pts[j] = p
            j+=1
    return pts,errs2,errs3,arrs
def get_best_combination(crit):
    ''' get best camera combination based on criterium "crit" (err2D or err3D)'''
    c_best = 1000
    i_best = 0
    for i,c in crit.iteritems():
        if c < c_best:
            c_best = c
            i_best = i
    return i_best, c_best
def save_combinations(out_dir,fname,arrs,pts,errs,errors='',title=''):
    '''
    Saves obtained 2D and 3D errors in .png and textfile.

    _Parameters_:

    out_dir: output directory

    fname: file name

    arrs: list of used camera combinations

    pts: array with estimated positions for all camera combinations

    errs: array with errors for all camera combinations

    _Returns_:

    Nothing

    '''
    fname2 = fname+".txt"
    with open(out_dir+fname2,"w") as f:
        f.write("{0:20}\t{1:5}\t{2:5}\t{3:5}\t{4}\n".format("combi","x","y","z",errors))
        for i,arr in arrs.iteritems():
            pt = pts[i]
            for ar in arr:
                f.write("{0:20}\t".format(ar))
            f.write("{0:5.0f}\t{1:5.0f}\t{2:5.0f}\t".format(pt[0,0],pt[1,0],pt[2,0]))
            for err in errs:
                f.write("{0:5.0f}\t".format(err.values()[i]))
            f.write("\n")

    # prepare for plotting
    fix,ax1 = plt.subplots()
    j = 0
    linestyles=['--','-',':']
    for err in errs:
        ax1.plot(err.values(),linestyles[j],color='b')
        j+=1
    ax1.legend(('2D fixed','2D free','3D free'),loc='best')
    ax1.set_xlabel('Camera Combinations')
    ax1.set_ylabel('Error [mm]',color='b')
    for tl in ax1.get_yticklabels():
        tl.set_color('b')
    ax1.set_title(title)
    plt.xticks(arrs.keys(),arrs.values())#or degrees
    # add space at bottom so that xticks not cut off
    plt.subplots_adjust(bottom=0.4)
    # turn xticks
    plt.setp(ax1.xaxis.get_majorticklabels(), rotation=70)
    plt.show(block=False)
    plt.savefig(out_dir+fname)
def triangulate(P1,P2,i1,i2,p_real=''):
    '''
    Performs triangulation with cv2-function triangulatePoints.

    _Parameters_:

    P1,P2: camera Projection matrixes

    i1,i2: point image positions in 2 cameras

    p_real: real position of point (if known), for error calculation

    _Returns_:

    position of corresponding 3D point, 2D and 3D error.
    '''
    p_test = cv2.triangulatePoints(P1,P2,i1,i2)
    p_norm = p_test/p_test[3]
    p_norm = p_norm.T[:,:3]
    if type(p_real)!=str:
        err3 = np.sqrt(np.sum(np.power(p_real-p_norm,2),axis=1))
        err2 = np.sqrt(np.sum(np.power(p_real[:,:2]-p_norm[:,:2],2),axis=1))
    return p_norm,err2,err3
def change_wall_to_ref(pts_basis,margin,pt_wall):
    '''
    Change of basis from wall to reference points.

    _Parameters_: (all distances in mm)

    pts_basis: list of positions of first and second reference points in wall

    reference. ([x1,y1],[x2,y2])

    margin: margin from leftmost and downmost reference points to new reference.

    pt_wall: position of point in wall reference

    _Returns_:

    position of point in reference point basis (in meters)
    '''
    pt_ref = pt_wall.copy()
    pt1 = pts_basis[0]
    pt2 = pts_basis[1]
    pt_w = pt_wall[0,:2].reshape(2,1)
    t_vec=pt1.reshape(2,1)
    theta = -np.arctan((pt1[1]-pt2[1])/(pt1[0]-pt2[0]))
    rotation = np.matrix(([np.cos(theta),-np.sin(theta)],
                        [np.sin(theta),np.cos(theta)]))
    pt_r = rotation*np.matrix(pt_w - t_vec)+np.matrix(margin).reshape(2,1)
    pt_ref[0,:2]=pt_r.reshape(1,2)
    return pt_ref
def change_ref_to_wall(pts_basis,margin,pt_ref):
    ''' inverse of change_wall_to_ref '''
    pt_wall = pt_ref.copy()
    pt1 = pts_basis[0]
    pt2 = pts_basis[1]
    pt_r = pt_ref[0,:2].reshape(2,1)
    t_vec=pt1.reshape(2,1)
    theta = -np.arctan((pt1[1]-pt2[1])/(pt1[0]-pt2[0]))
    rotation = np.matrix(([np.cos(theta),-np.sin(theta)],
                        [np.sin(theta),np.cos(theta)]))
    pt_w = rotation.I*(pt_r - np.matrix(margin).reshape(2,1))+t_vec
    pt_wall[0,:2]=pt_w.reshape(1,2)
    return pt_wall
def plot_geometry(pts_basis,margin,real_array=[],obj_array=[],fname=''):
    plt.figure()
    # plot reference points
    plt.plot(pts_basis[:,0],pts_basis[:,1],color='k',linestyle='-')
    # plot reference coordinate system
    origin=change_ref_to_wall(pts_basis,margin,np.matrix([0,0,0]))
    xaxis=change_ref_to_wall(pts_basis,margin,np.matrix([margin[1],0,0]))
    yaxis=change_ref_to_wall(pts_basis,margin,np.matrix([0,margin[1],0]))
    plt.plot([yaxis[0,0],origin[0,0],xaxis[0,0]],[yaxis[0,1],origin[0,1],xaxis[0,1]],color='b',linestyle=':')
    # plot robot positions
    if real_array!=[]:
        plt.plot(real_array[:,0],real_array[:,1],marker='o',color='g')
    if obj_array!=[]:
        plt.plot(obj_array[:,0],obj_array[:,1],marker='o',color='r')
    plt.legend(['checkerboard base line','reference basis',
                'robot real positions','robot calculated positions'],'best')


    # plot properties
    plt.grid(True)
    plt.axes().set_aspect('equal','datalim')
    plt.show(block=False)

    if fname!='':
        plt.savefig(fname)
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

        _Returns_: Nothing
    '''
    print("Writing ref positions to ",dirname+name)
    with open(dirname+name,"w") as f:# overwrite
        for pt in pts_img:
            f.write(str(pt[0])+"\t"+str(pt[1])+"\n")
        for m in M:
            f.write(str(m[0,0])+"\t"+str(m[0,1])+"\t"+str(m[0,2])+"\n")
        for pt in pts_obj:
            f.write(str(pt[0])+"\t"+str(pt[1])+"\n")
def write_pos(dirname,name,p):
    ''' Save robot position (img,obj or real) in file, appending to old results

        _Parameters_:

        dirname = directory

        name = name of file

        p = np array with robot position (x,y)

        _Returns_: Nothing'''
    with open(dirname+name,"a") as f: # append
        # if p.any():
            # for pt in p:
                # f.write(str(pt)+'\t')
            # f.write("\n")
        if p.any():
            if p.shape[0]==2:
                f.write("{0}\t{1}\n".format(p[0,0],p[1,0]))
            else:
                f.write("{0}\t{1}\t{2}\n".format(p[0,0],p[0,1],p[0,2]))

class Camera:
    '''
    This class contains the camera's intrinsic and extrinsic parameters
    and the functions used for calibration.
    '''
    def __init__(self,n,rms=0,C=0,dist=0,r=0,t=0):
        ## Camera IP number
        self.n = n
        ## rms error from intrinsic calibration
        self.rms=rms
        ## Camera matrix from intrinsic calibration
        self.C = np.matrix(C)
        ## Distortion coefficients
        self.dist = np.matrix(dist)
        ## Rotation vector
        self.r = np.matrix(r)
        ## Translation vector
        self.t = np.matrix(t)
        ## Camera rotation matrix
        self.R = ''
        ## Projection matrix [R,t]
        self.Proj= ''
        ## Camera centers in object reference frame
        self.Center = ''
    def update(self):
        ''' Update R (Camera rotation matrix), Proj (Projection matrix [R,t]
        and the Center coordinates in object reference frame'''
        R,__ = cv2.Rodrigues(self.r)
        self.R = np.matrix(R)
        self.Proj = self.C*np.hstack((self.R,self.t))
        self.Center = -self.R.I*self.t
    def read(self,in_dir,fisheye=False):
        ''' get camera parameters from file '''
        import numpy as np
        i=0
        rms = 0.0
        cam = np.zeros((3,3))
        dist = np.zeros((1,5))
        rvecs = []
        tvecs = []
        fname = "results"
        if fisheye==True:
            fname = "results_fish"
        print("Loading camera parameters from",in_dir+fname+str(self.n)+".txt")
        with open(in_dir+fname+str(self.n)+".txt","r") as f:
            c = csv.reader(f,delimiter='\t')
            count = sum(1 for _ in c) # number of lines in file
            dim = int((count-5)/2) #number of images in folder
            f.seek(0) # go back to beginning of file
            c = csv.reader(f,delimiter='\t')
            for line in c:
                if i == 0:
                    rms = line
                elif i >= 1 and i <= 3:
                    cam[i-1,:] = line
                elif i == 4:
                    dist = [float(x) for x in line if x!='']
                elif i>=5 and i<5+dim:
                    rvecs.append([float(x) for x in line if x!=''])
                elif i>=5+dim and i <5+2*dim:
                    tvecs.append([float(x) for x in line if x!=''])
                i+=1
        self.rms = rms
        self.C = np.matrix(cam)
        self.dist = np.array(dist)
        self.r = np.matrix(rvecs)
        self.t = np.matrix(tvecs)
    def save(self,in_dir,fisheye):
        ''' save camera parameters in file '''
        fname = "results"
        if fisheye:
            fname = "results_fish"
        with open(in_dir+fname+str(self.n)+".txt","w") as f:
            f.write(str(self.rms)+'\n')
            for e in self.C:
                f.write("{0:10.4f}\t{1:10.4f}\t{2:10.4f}\n".format(e[0,0],e[0,1],e[0,2]))
            for e in self.dist[0]:
                f.write("{0:10.4f}\t".format(e))
            f.write("\n")
            for j in range(len(self.r.T)):
                for i in range(3):
                    f.write("{0:10.4f}\t".format(self.r.T[j,i]))
                f.write("\n")
            for j in range(len(self.t.T)):
                for i in range(3):
                    f.write("{0:10.4f}\t".format(self.t.T[j,i]))
                f.write("\n")
    def save_Center(self,in_dir,fname):
        self.update()
        with open(in_dir+fname+".txt","a") as f:
            f.write("{0}\t{1:10.4f}\t{2:10.4f}\t{3:10.4f}\t".format(self.n,
                                                                self.Center[0,0],
                                                               self.Center[1,0],
                                                               self.Center[2,0]))
            f.write("\n")

    def get_checkpoints(self,out_dir,w,h,fisheye):
        '''
        gets checkboard points for the intrinsic camera  calibration.
        '''
        choice = "y"
        counter = 1
        obj_points = []
        img_points = []
        pattern_size = (w, h)
        while True:
            try:
                # Collect Data
                if choice == "y":
                    img = get.get_image(self.n)
                    h, w = img.shape[:2]
                    img_pt, obj_pt,__ = self.get_calibpoints(img,pattern_size,
                                                     counter,out_dir,fisheye)
                    if not obj_pt == []:
                        img_points.append(img_pt)
                        obj_points.append(obj_pt)
                        counter += 1
                    choice = raw_input("Do you want to take another image? (y/n)")
                elif choice == "n":
                    return img_points, obj_points, (w,h)
                else:
                    choice = raw_input("Enter valid choice (y/n)")
            except KeyboardInterrupt:
                print("program terminated by user")
                sys.exit(1)
    def get_calibpoints(self,img,pattern_size,counter,out_dir,fisheye,save_input=True,save_output=True):
        '''
        returns pairs of object- and image points (chess board corners)
        for camera calibration
        '''
        square_size = 140 # in mm
        #print("square size:",square_size)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        pattern_points = np.zeros( (np.prod(pattern_size), 3), np.float32 )
        pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
        pattern_points *= square_size

        h, w = 0, 0
        try:
            found, corners = cv2.findChessboardCorners(gray, pattern_size)
        except KeyboardInterrupt:
            sys.exit(1)
        fig,ax=plt.subplots()
        if found:
            print(str(counter)+": chessboard found")
            term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1 )
            cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), term)
            if out_dir:
                print("saving...")
                out = img.copy()
                cv2.drawChessboardCorners(out, pattern_size, corners, found)
                ax.imshow(out),plt.show(block=False)
                fname = "input"
                if int(fisheye[int((self.n-139)/2)]):
                    fname="input_fish"
                if save_input:
                    cv2.imwrite('{0}/{1}{2}_{3}.jpg'.format(out_dir,fname,self.n,counter),img)
                fname = "output"
                if int(fisheye[int((self.n-139)/2)]):
                    fname="output_fish"
                if save_output:
                    cv2.imwrite('{0}/{1}{2}_{3}.jpg'.format(out_dir,fname,self.n,counter), out)
            return corners.reshape(-1, 2), pattern_points,out
        elif not found:
            if not corners == None:
                print(str(counter)+': chessboard only partially found')
                out = img.copy()
                cv2.drawChessboardCorners(out, pattern_size, corners, found)
                ax.imshow(out),plt.show(block=False)
                fname="output"
                if int(fisheye[int((self.n-139)/2)]):
                    fname="output_fish"
                cv2.imwrite('{0}/{1}{2}_{3}.jpg'.format(out_dir,fname,self.n,counter), out)
            else:
                ax.imshow(img),plt.show(block=False)
                print(str(counter)+': chessboard not found')
            return [],[],0
    def save_checkpoints_file(self,out_dir,n_cameras):
        '''
        take pictures and save them in out_dir for later processing
        '''
        choice = "y"
        counter = 1
        while True:
            try:
                if choice == "y":
                    for n in n_cameras:
                        img = get.get_image(n)
                        cv2.imwrite('{0}/input{1}_{2}.jpg'.format(out_dir,n,counter),img)
                    counter += 1
                    choice = raw_input("Do you want to take another image? (y/n)")
                elif choice == "n":
                    return counter
                else:
                    choice = raw_input("Enter valid choice (y/n)")

            except KeyboardInterrupt:
                print("program terminated by user")
                sys.exit(1)
    def get_checkpoints_file(self,out_dir,w,h,fisheye,img=''):
        '''
        gets checkboard points for the intrinsic camera calibration from pictures
        that have been taken previously (either in file structure or given
        as argument)
        '''
        choice = "y"
        counter = 1
        obj_points = []
        img_points = []
        pattern_size = (w, h)
        try:
            # Collect Data from input files in folder
            if img=='':
                for f in os.listdir(out_dir):
                    if f.startswith("input"+str(self.n)):
                        img = cv2.imread(out_dir+f,cv2.IMREAD_COLOR)
                        print("reading ",out_dir+f)
                        h, w = img.shape[:2]
                        img_pt, obj_pt,__ = self.get_calibpoints(img,pattern_size,
                                                              counter,out_dir,fisheye,0)
                        if not obj_pt == []:
                            img_points.append(img_pt)
                            obj_points.append(obj_pt)
                        counter += 1
            # work with image given as argument
            else:
                h, w = img.shape[:2]
                img_pt, obj_pt,output = self.get_calibpoints(img,pattern_size,
                                                      counter,out_dir,fisheye,0)
            if not obj_pt == []:
                img_points.append(img_pt)
                obj_points.append(obj_pt)
            return img_points,obj_points,output
        except KeyboardInterrupt:
            print("Program terminated by user")
            sys.exit(1)
    def calibrate(self,obj_points,img_points,size):
        rms,C,dist,r,t = cv2.calibrateCamera(obj_points,img_points,size, None, None)
        self.rms = rms
        self.C = np.matrix(C)
        self.dist = np.array(dist)
        self.r = np.matrix(r[0])
        self.t = np.matrix(t[0])
    def reposition(self,opts,ipts,flag=cv2.CV_P3P,ransac=0,err=8):
        '''
        Redefines the position vectors r and t of the camera given a set of
        corresponding image and object points
        Options:
            -guess for camera position (using rvguess function)
            -method: Iterative = 0, EPNP = 1, P3P = 2
            -ransac method for solvepnp
        '''
        rvec = 0
        tvec = 0
        if flag == cv2.CV_P3P or flag == cv2.CV_EPNP:
            n_points = ipts.shape[0]
            ipts = np.ascontiguousarray(ipts[:,:2]).reshape((n_points,1,2))
        if ransac:
            rvec,tvec,result = cv2.solvePnPRansac(opts,ipts,self.C,self.dist,
                                                  flags=flag,reprojectionError=err)
        else:
            result,rvec,tvec = cv2.solvePnP(opts,ipts,self.C,self.dist,
                                        self.r,self.t,0,flag)
        self.r = np.matrix(rvec)
        self.t = np.matrix(tvec)
        self.update()
        return result
    def reposition_homography(self,opts,ipts):
        '''
        calculate Proj matrix following instructions of
        'A Flexible New Technique for Camera
        Calibration' of Zhang, 2000.
        '''
        opts = np.array(opts[:2,:])
        ipts = np.array(ipts)
        [h1,h2,h3],__ = cv2.findHomography(opts,ipts,0)
        [h1,h2,h3] = np.matrix([h1,h2,h3])
        lamda1 = 1/np.linalg.norm(self.C.I*h1.T)
        lamda2 = 1/np.linalg.norm(self.C.I*h2.T)
        lamda = np.mean([lamda1,lamda2])
        r1 = lamda*self.C.T*h1.T
        r2 = lamda*self.C.T*h2.T
        r3 = np.cross(r1.T,r2.T)
        r3 = r3.T
        t = lamda*self.C.T*h3.T
        proj = np.hstack((r1,r2,r3,t))
    def check_imagepoints(self,p_obj,p_img):
        '''
        Returns images of reference points (known) and the relative error matrix
        '''
        test_img = self.Proj*p_obj.T
        lamda = test_img[2,:]
        test_img = test_img/lamda
        test_img=test_img[:2,:].T
        #match_matrix=(p_img-test_img)/test_img
        err_img=np.sqrt(np.sum(np.power(test_img[:,:2]-p_img[:,:2],2),axis=1))
        return test_img,err_img,lamda
    def check_objectpoints(self,p_obj,p_img):
        # From least squares
        A =(self.Proj.T*self.Proj).I*self.Proj.T
        x_tilde = self.C.I*p_img.T
        test_obj =A*x_tilde

        #test_obj2 = self.Proj.I*self.C.I*p_img.T

        # eliminate last column that seems to screw everything up
        #test_obj3 = A[:,:2]*(cam.C.I*p_img.T)[:2,:]

        test_obj = test_obj/test_obj[3,:]
        err_obj=np.sqrt(np.sum(np.power(test_obj.T[:,:3]-p_obj[:,:3],2),axis=1))
        return test_obj, err_obj
    def get_theta(self):
        __,__,__,x,y,z,euler =cv2.decomposeProjectionMatrix(self.Proj)
        #self.yaw = np.arccos(x[1,1])*180/np.pi #yaw
        #self.pitch = np.arccos(y[0,0])*180/np.pi #pitch
        #self.roll = np.arccos(z[0,0])*180/np.pi #roll
        self.yaw=euler[0]
        self.pitch=euler[1]
        self.roll=euler[2]
    def ransac_loop(self,img,flag,repErr_range):
        ''' Loops through values for reprojection Error tolerance (from repErr_range)
        and finds best fit based on error of reprojected reference points.
        '''
        #TODO: find best fit based on error of reference points Z-Match?
        err_best = 0
        repErr_best = 0

        sum_max = 1000
        for repErr in repErr_range:
            result = self.reposition(img.ref_obj,img.ref_img,flag,1,repErr)
            # Check only chosen refpoints by Ransac
            ref_img = ref_obj = 0
            try:
                ref_img = img.ref_img[result].reshape(result.shape[0],2)
                ref_obj = img.ref_obj[result].reshape(result.shape[0],3)
                # Check if the match is new best match
                ref, err_img,__ = self.check_imagepoints(img.augment(ref_obj),ref_img)
                sum_current = err_img.sum()/err_img.shape[0]
                if sum_current < sum_max:
                    sum_max = sum_current
                    repErr_best = repErr
                    err_best=err_img
                    print('best error',sum_max)
                    print('RANSAC repErr:',repErr,' inliners: ',result.T)
            except:
                print(repErr,': no result obtained')
                next
        # Use best found repErr to reposition camera
        self.reposition(img.ref_obj,img.ref_img,flag,1,repErr_best)
        return err_best
    def undistort(self,img):
        # intrinsic parameters
        fx = self.C[0,0]
        fy = self.C[1,1]
        cx = self.C[0,2]
        cy = self.C[1,2]
        # radial distortion coefficients
        k1,k2 = self.dist[0:2]
        # †angential distortion coefficients
        p1,p2 = self.dist[2:4]
        intrinsics = cv.CreateMat(3, 3, cv.CV_64FC1)
        cv.Zero(intrinsics)
        intrinsics[0, 0] = float(fx)
        intrinsics[1, 1] = float(fy)
        intrinsics[2, 2] = 1.0
        intrinsics[0, 2] = float(cx)
        intrinsics[1, 2] = float(cy)

        dist_coeffs = cv.CreateMat(1, 4, cv.CV_64FC1)
        cv.Zero(dist_coeffs)
        dist_coeffs[0, 0] = float(k1)
        dist_coeffs[0, 1] = float(k2)
        dist_coeffs[0, 2] = float(p1)
        dist_coeffs[0, 3] = float(p2)

        #src = cv.LoadImage(src)
        src = cv.fromarray(img)
        dst = img.copy()
        dst = cv.fromarray(dst)
        #dst = cv.CreateImage(cv.GetSize(src), src.type, src.channels)
        mapx = cv.CreateImage(cv.GetSize(src), cv.IPL_DEPTH_32F, 1)
        mapy = cv.CreateImage(cv.GetSize(src), cv.IPL_DEPTH_32F, 1)
        cv.InitUndistortMap(intrinsics, dist_coeffs, mapx, mapy)
        cv.Remap(src, dst, mapx, mapy, cv.CV_INTER_LINEAR + cv.CV_WARP_FILL_OUTLIERS,  cv.ScalarAll(0))
        # cv.Undistort2(src,dst, intrinsics, dist_coeffs)
        return np.array(dst)

class Image:
    '''
    The class Image contains all image- and object space positions of the
    reference points and the robot location in the respective images.
    '''
    def __init__(self,n,ref_obj=0,r_truth=0,ref_img=0,ref=0,r_img=0,r=0):
        ## Camera IP number
        self.n = n
        ## reference image points
        self.ref_img = ref_img
        ## reference object points
        self.ref_obj = ref
        ## robot image point
        self.r_img = r_img
        ## robot object point
        self.r_obj = r
        ## robot real position
        self.r_truth = r_truth
    def take_image(self):
        ''' Get image from camera'''
        print("Taking image from camera",self.n,"(this can take a moment)...")
        self.img = get.get_image(self.n)
    def load_image(self,fname,swop=True):
        ''' Load image from file.

        _Parameters_:

        fname: path to picture to be loaded

        swop: if True, picture will be converted from BGR(cv2 default) to RGB (matplotlib default)

        _Returns_:

        None (updates self.img)

        '''
        print('loading image from',fname)
        img=cv2.imread(fname)
        if swop:
            b,g,r= cv2.split(img)
            self.img=cv2.merge((r,g,b))
        else:
            self.img=img
    def show_hsv(self):
        h,s,v = cv2.split(cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV))
        plt.close(1),plt.close(2),plt.close(3)
        plt.figure(1),plt.imshow(h),plt.colorbar(),plt.show(block=False)
        plt.figure(2),plt.imshow(s),plt.colorbar(),plt.show(block=False)
        plt.figure(3),plt.imshow(v),plt.colorbar(),plt.show(block=False)
    def get_newest(self,dirname,fname):
        ''' Get newest file in directory "dirname" starting with
        "fname" and camera number

        _Parameters_:

        dirname: directory where file is to be searched for.

        fname: start of filename (will be followed by camera number)

        _Returns_:

        Full relative path to newest file

        '''
        f_newest = '0000000000'
        for f in os.listdir(dirname):
            if f.startswith(fname+str(self.n)):
                if int(f[-14:-4]) > int(f_newest[-14:-4]):
                    f_newest = f
        return dirname+f_newest
    def read_ref(self,dirname,fname,m):
        '''
        Get reference point img positions from newest file in directory
        'dirname'
        '''
        fname = self.get_newest(dirname,fname)
        print("Loading ref positions from ",fname)
        i = 0
        pts_img = np.zeros((m,2))
        M = np.zeros((3,3))
        pts_obj = np.zeros((m,2))
        with open(fname,"r") as f:
            c = csv.reader(f,delimiter = '\t')
            for line in c:
                if i >= 0 and i <m:
                    pts_img[i,:] = line
                elif i >= m and i < m+3:
                    M[i-m,:] = line
                elif i >= m+3 and i<2*m+3:
                    pts_obj[i-(m+3),:] = line
                i+=1
        img_points = pts_img.astype(np.float32)
        obj_points = pts_obj.astype(np.float32)
        self.ref_img = np.matrix(img_points)
        self.ref_obj = np.matrix(obj_points)
        self.M = M
    def read_pos(self,dirname,fname):
        '''
        Get newest robot position from pos_img file of this camera
        '''
        fname = self.get_newest(dirname,fname)
        #fname=dirname+fname+str(self.n)+".txt"
        p = ''
        with open(fname,"r") as f:
            c = csv.reader(f,delimiter = '\t')
            for line in c:
                p = [float(x) for x in line if x!='']
        # last value of p corresponds to last line
        self.r_img = np.array(p,dtype=np.float32)
    def augment(self,pt,z = ''):
        ''' Add third dimension to points (ones if not specified) '''
        dim = pt.shape[0]
        if z=='':
            try:
                return np.matrix(np.hstack((pt,1)))
            except:
                return np.matrix(np.hstack((pt,np.ones((dim,1))))).astype(np.float32)
        else:
            return np.matrix(np.hstack((pt,z.reshape(dim,1)))).astype(np.float32)
    def get_robotimage(self,R,THRESH,MIN,MAX,save=0,out_dir='',TIME=0,loop=''):
        ''' Finds robot position in image using perspective.imagepoints().

        _Parameters_:

        R,THRESH,MIN,MAX: parameters used by perspective.imagepoints().

        save: set to 1 if you wish to save results (images)

        out_dir: directory where to save images.

        TIME: start time of program (for file naming)

        _Returns_:

        Nothing
        '''
        img_red,circ_red,p,__ = persp.imagepoints(self.img,R,1,THRESH,MIN,MAX)
        self.r_img=p
        if save:
            imgs={'img_red_'+str(self.n)+'_'+TIME:img_red,
                  'circ_red_'+str(self.n)+'_'+TIME:circ_red,
                  'img_'+str(self.n)+'_'+TIME:self.img}
            persp.visualization(imgs,self.n)
            persp.save_open_images(out_dir,loop)
            plt.close('all')
    def get_refimage(self,R,THRESH,MIN,MAX,NPTS,save=0,out_dir='',TIME=0,loop=''):
        ''' Finds reference position in image using perspective.imagepoints()

        _Parameters_:

        R,NPTS,THRESH,MIN,MAX: parameters used by perspective.imagepoints()

        save: set to 1 if you wish to save results (images)

        out_dir: directory where to save images.

        TIME: start time of program (for file naming)

        _Returns_:

        Nothing
        '''
        img_org,circ_org,pts_img,th_org = persp.imagepoints(self.img,R,NPTS,THRESH,
                                                            MIN,MAX)
        self.ref_img=pts_img
        if save:
            h,s,__ = cv2.split(cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV))
            imgs={'img_h_'+str(self.n)+'_'+TIME:h,'img_s_'+str(self.n)+'_'+TIME:s}
            persp.visualization(imgs,self.n,1)
            imgs={'img_org_'+str(self.n)+'_'+TIME:img_org,
                  'circ_org_'+str(self.n)+'_'+TIME:circ_org,
                  'img_'+str(self.n)+'_'+TIME:self.img}
            persp.visualization(imgs,self.n)
            persp.save_open_images(out_dir,loop)
            plt.close('all')
    def get_refobject(self,input_obj,NPTS,MARGIN,save=0,out_dir='',TIME=0):
        pts_obj,M = persp.objectpoints(NPTS,input_obj)
        __,pts_obj,size = persp.format_points(pts_obj,MARGIN)
        img_flat,M = persp.geometric_transformationN(self.img,pts_obj,self.ref_img,size)
        self.ref_obj = pts_obj
        self.M = M
        if save:
            # images
            img_summary = persp.create_summary(img_flat,pts_obj)
            imgs = {'summary_'+str(self.n)+'_'+TIME:img_summary}
            persp.visualization(imgs,self.n,0,1)
            persp.save_open_images(out_dir)
            plt.close('all')
    def get_checkerboard(self,in_dir,fisheye,w,h,MARGIN,
                         R,THRESH,MIN,MAX,save,out_dir='',TIME=0,loop=''):
        cam = Camera(self.n)
        cam.read(in_dir,fisheye)
        ipts,opts,img_out=cam.get_checkpoints_file(out_dir,w,h,fisheye,self.img)
        plt.close('all')
        if len(ipts) == 0:
            return 0

        img_org,circ_org,pts_img,__ = persp.imagepoints(self.img,R,3,THRESH,MIN,MAX)
        A = pts_img[0]
        B = pts_img[1]
        C = pts_img[2]
        a = np.array([10000,10000])
        b = np.array([10000,10000])
        c = np.array([10000,10000])
        min_a=10000
        min_b=10000
        min_c=10000
        a_ind = b_ind = c_ind = 0
        for i,pt in enumerate(ipts[0]):
            if np.linalg.norm(pt-A)<min_a:
                min_a = np.linalg.norm(pt-A)
                a=pt
                a_ind = i
            if np.linalg.norm(pt-B)<min_b:
                min_b = np.linalg.norm(pt-B)
                b=pt
                b_ind = i
            if np.linalg.norm(pt-C)<min_c:
                min_c = np.linalg.norm(pt-C)
                c=pt
                c_ind = i
        ''' reorder points from a to b and a to c '''
        ipts_new=np.zeros(ipts[0].shape,dtype=np.float32)
        j = np.zeros((ipts[0].shape[0],1))
        index = np.zeros((ipts[0].shape[0],1))
        for i in range(ipts[0].shape[0]):
            j[i] = i + 1 - np.mod(i,w)
            index[i]=a_ind+(c_ind-a_ind)/(w*(h-1))*(j[i]-1)+(b_ind-a_ind)/(w-1)*(i+1-j[i])
            ipts_new[i,:] = ipts[0][int(index[i]),:]

        ''' choose 4 corner points for homography'''
        ipts_selec = np.array([ipts_new[0],ipts_new[w-1],
                                ipts_new[w*(h-1)],ipts_new[w*h-1]])
        opts_selec = np.array([opts[0][0],opts[0][w-1],
                                 opts[0][w*(h-1)],opts[0][w*h-1]])

        pts_obj = opts_selec.copy()
        pts_obj[:,0]+=MARGIN[0]
        pts_obj[:,1]+=MARGIN[1]
        size = np.amax(pts_obj[:,:2],axis=0)+MARGIN
        size = (int(size[0]),int(size[1]))
        img_flat,M = persp.geometric_transformationN(self.img,pts_obj[:,:2],
                                                     ipts_selec,size)

        ref_obj=opts[0].copy()
        ref_obj[:,0]+=MARGIN[0]
        ref_obj[:,1]+=MARGIN[1]
        self.ref_img = ipts_new
        self.ref_obj = ref_obj
        self.M = M

        if save:
            img_summary = persp.create_summary(img_flat,pts_obj)
            imgs={'summary_'+str(self.n)+'_'+TIME:img_summary}
            persp.visualization(imgs,self.n,0,1)
            imgs={'img_org_'+str(self.n)+'_'+TIME:img_org,
                'circ_org_'+str(self.n)+'_'+TIME:circ_org,
                  'img_out_'+str(self.n)+'_'+TIME:img_out}
            persp.visualization(imgs,self.n)
            persp.save_open_images(out_dir)

        return 1
