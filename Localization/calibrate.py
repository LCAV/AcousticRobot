#-*- coding: utf-8 -*-
#!/usr/bin/env python
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
USAGE = '''
USAGE: calib.py [-o <output path>] [-c <camera path>]
[-n <camera_number>] [-f <fisheye on (0/1)>]
'''

import numpy as np

def combinations(array,number):
    ''' returns vector with combinations of "number" elements out of "array" '''
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
    cam_dir = ''
    n = 141
    fisheye = 0
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
            cam_dir = str(arg)
        if opt in ('--n',"-n"):
            n = int(arg)
        if opt in ('--fisheye',"-f"):
            fisheye = int(arg)
    if out_dir[-1]!="/":
        out_dir = out_dir+"/"
    if cam_dir[-1]!="/":
        cam_dir = cam_dir + "/"
    return out_dir,cam_dir,n,fisheye
def get_calibpoints(img,pattern_size,counter,out_dir,n,fisheye):
    '''
    returns pairs of object- and image points (chess board corners)
    for camera calibration'''
    square_size = 47 # in mm
    square_size = square_size/px_size # †his doesn't matter for camera calibration!!
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
            if fisheye:
                fname="input_fish"
            cv2.imwrite('{0}/{1}{2}_{3}.jpg'.format(out_dir,fname,n,counter),img)
            fname = "output"
            if fisheye:
                fname="output_fish"
            cv2.imwrite('{0}/{1}{2}_{3}.jpg'.format(out_dir,fname,n,counter), out)
        return corners.reshape(-1, 2), pattern_points
    elif not found:
        if not corners == None:
            print(str(counter)+': chessboard only partially found')
            out = img.copy()
            cv2.drawChessboardCorners(out, pattern_size, corners, found)
            ax.imshow(out),plt.show(block=False)
        else:
            ax.imshow(img),plt.show(block=False)
            print(str(counter)+': chessboard not found')
        return [],[]
def get_leastsquares(cam,pt,method='hz',r_height='',p_real=''):
    '''
    Solve least squares problem with or without fixed height
    Returns real position of robot based on observation with cam1 and cam2
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
        ''' Own Method using algebraic transformations '''
        # Not quite right because scaling factor not taken into account!_part[i] = np.vstack((x*f3-f1,y*f3-f2))
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

def get_bestcameras(n_cameras,cam,pt,number,method='hz',r_height='',p_real=''):
    array=combinations(n_cameras,3)
    cams = dict()
    pts_ref = dict()
    errs = dict()
    err2_best = 100
    err3_best = 100
    p_best = 0
    arr_best = 0

    for i,arr in enumerate(array):
        cams = [x for x in cam if x.n in arr]
        pts_ref = [x for i,x in enumerate(pt) if i*2+139 in arr]
        p,err2,err3 = get_leastsquares(cams,pts_ref,method,r_height,p_real)
        errs[arr] = err3
        if err3 < err3_best:
            err3_best = err3
            err2_best = err2
            p_best = p
            arr_best = arr
    return p_best,err2_best,err3_best,arr_best,errs

def get_rvguess(rv,tv,i=2):
    global n
    ''' get guess for rv, tv from specific object points '''
    if n == 139: # vertical image of chessboard
        i = 2  # for test_friday only
    elif n == 141:
        i = 9 # for test_friday only
    rv = rv[i-1]
    tv = np.mean(tv[:8,2])
    return rv, tv
    p_norm2 = p_norm2.T[:,:3]
def triangulate(P1,P2,i1,i2,p_real):
    p_test = cv2.triangulatePoints(P1,P2,i1,i2)
    p_norm = p_test/p_test[3]
    p_norm = p_norm.T[:,:3]
    if type(p_real)!=str:
        err3 = np.sqrt(np.sum(np.power(p_real-p_norm,2),axis=1))
        err2 = np.sqrt(np.sum(np.power(p_real[:,:2]-p_norm[:,:2],2),axis=1))
    return p_norm,err2,err3

class Camera:
    def __init__(self,n,rms=0,C=0,dist=0,r=0,t=0):
        # “raw parameters"
        self.n = n
        self.rms=rms
        self.C = np.matrix(C)
        self.dist = np.matrix(dist)
        self.r = np.matrix(r)
        self.t = np.matrix(t)
    def update(self):
        ''' Update R,Proj and Center'''
        R,__ = cv2.Rodrigues(self.r)
        self.R = np.matrix(R)
        self.Proj = self.C*np.hstack((self.R,self.t))
        self.Center = -self.R.I*self.t
    def read(self,cam_dir,fisheye):
        ''' get camera parameters from file '''
        import numpy as np
        i=0
        rms = 0.0
        cam = np.zeros((3,3))
        dist = np.zeros((1,5))
        rvecs = []
        tvecs = []
        fname = "results"
        if fisheye:
            fname = "results_fish"
        with open(cam_dir+fname+str(self.n)+".txt","r") as f:
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
    def save(self,cam_dir,fisheye):
        ''' save camera parameters in file '''
        fname = "results"
        if fisheye:
            fname = "results_fish"
        with open(cam_dir+fname+str(self.n)+".txt","w") as f:
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
    def get_checkpoints(self,out_dir,fisheye):
        '''
        gets checkboard points for the intrinsic camera  calibration.
        '''
        choice = "y"
        counter = 1
        obj_points = []
        img_points = []
        pattern_size = (8, 5)
        while True:
            try:
                # Collect Data
                if choice == "y":
                    img = get.get_image(self.n)
                    h, w = img.shape[:2]
                    img_pt, obj_pt = get_calibpoints(img,pattern_size,
                                                     counter,out_dir,self.n,fisheye)
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
    def save_checkpoints_file(self,out_dir,n_cameras):
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
    def get_checkpoints_file(self,out_dir,fisheye):
        '''
        gets checkboard points for the intrinsic camera  calibration. elf.Proj.T*self.Proj).I*self.Proj.TROM FILES
        '''
        choice = "y"
        counter = 1
        obj_points = []
        img_points = []
        pattern_size = (8, 5)
        try:
            # Collect Data
            for f in os.listdir(out_dir):
                if f.startswith("input"+str(self.n)):
                    img = cv2.imread(out_dir+f,cv2.IMREAD_COLOR)
                    print("reading ",out_dir+f)
                    h, w = img.shape[:2]
                    img_pt, obj_pt = get_calibpoints(img,pattern_size,
                                                    counter,out_dir,self.n,fisheye)
                    if not obj_pt == []:
                        img_points.append(img_pt)
                        obj_points.append(obj_pt)
                    counter += 1
            return img_points, obj_points, (w,h)
        except KeyboardInterrupt:
            print("program terminated by user")
            sys.exit(1)
    def calibrate(self,obj_points,img_points,size):
        rms,C,dist,r,t = cv2.calibrateCamera(obj_points,img_points,size, None, None)
        self.rms = rms
        self.C = np.matrix(C)
        self.dist = np.array(dist)
        self.r = np.matrix(r[0])
        self.t = np.matrix(t[0])
    def reposition(self,opts,ipts,guess=0,flag=cv2.CV_P3P,ransac=0,err=8):
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
        if guess:
            tguess = {139:np.array([650,120]),
                       141:np.array([450,50])}
            r_guess,h_guess = get_rvguess(self.r,self.t*px_size)
            t_guess=np.hstack([tguess[self.n],h_guess])
            self.t = np.matrix(t_guess)
            self.r = np.matrix(r_guess)
        if ransac:
            rvec,tvec,result = cv2.solvePnPRansac(opts,ipts,self.C,self.dist,
                                                  flags=flag,reprojectionError=err)
        else:
            result,rvec,tvec = cv2.solvePnP(opts,ipts,self.C,self.dist,
                                        self.r,self.t,guess,flag)
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
        __,__,__,x,y,z,__ =cv2.decomposeProjectionMatrix(self.Proj)
        self.yaw = np.arccos(x[1,1])*180/np.pi #yaw
        self.pitch = np.arccos(y[0,0])*180/np.pi #pitch
        self.roll = np.arccos(z[0,0])*180/np.pi #roll
    def ransac_loop(self,img,flag,repErr_range):
        ''' Loops through values for reprojection Error tolerance (from repErr_range)
        and finds best fit based on error of reprojected reference points.
        '''
        #TODO: find best fit based on error of reference points Z-Match?
        err_best = 0
        repErr_best = 0

        sum_max = 1000
        for repErr in repErr_range:
            result = self.reposition(img.ref_real,img.ref_img,0,flag,1,repErr)
            # Check only chosen refpoints by Ransac
            ref_img = ref_real = 0
            try:
                ref_img = img.ref_img[result].reshape(result.shape[0],2)
                ref_real = img.ref_real[result].reshape(result.shape[0],3)
                # Check if the match is new best match
                ref, err_img,__ = self.check_imagepoints(img.augment(ref_real),ref_img)
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
        self.reposition(img.ref_real,img.ref_img,0,flag,1,repErr_best)
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
    def __init__(self,n,ref_real=0,r_truth=0,ref_img=0,ref=0,r_img=0,r=0):
        self.n = n
        self.ref_img = ref_img
        self.ref = ref
        self.r_img = r_img
        self.r = r
        self.r_truth = r_truth
        self.ref_real = ref_real
    def get_newest(self,dirname,fname):
        ''' Get newest file name '''
        f_newest = '0000000000'
        for f in os.listdir(dirname):
            if f.startswith(fname+str(self.n)):
                if int(f[-14:-4]) > int(f_newest[-14:-4]):
                    f_newest = f
        return dirname+f_newest
    def read_ref(self,dirname,fname,m):
        '''
        get reference point img positions from newest file in directory
        'dirname'
        '''
        fname = self.get_newest(dirname,fname)
        #print("reading",fname)
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
        self.ref_real = np.matrix(obj_points)
    def read_pos(self,dirname,fname):
        '''
        get newest robot position from pos_img file of this camera
        '''
        #fname = self.get_newest(dirname,fname)
        fname=dirname+fname+str(self.n)+".txt"
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

global px_size
px_size = 1.4*10**-3 # mm per pixel

if __name__ == '__main__':
    import getopt

   #------------------------- Get parameters -------------------------#
    out_dir,cam_dir,n,fisheye = get_param()
    #------------------------ Calibrate Camera ------------------------#

    # make new calibration
    #camera = Camera(n)
    #img_points,obj_points,size = camera.get_checkpoints(out_dir,fisheye)
    #root mean square (RMS) re-projection error (good between 0.1 and 1)
    #camera.calibrate(obj_points,img_points,size)
    #camera.save(out_dir)

    #-------------------------- Undistort   ---------------------------#

    #camera2.read(cam_dir)
    #for f in os.listdir(out_dir):
        #if f.startswith("input"):
        #if f.startswith("img"+str(camera2.n)):
            #img_in=cv2.imread(out_dir+f,cv2.IMREAD_COLOR)
            #undone = camera2.undistort(img_in)
            #cv2.imwrite(out_dir+f.replace("img","undone"),undone)
            #cv2.imwrite(out_dir+f.replace("input","undone"),undone)

    #------------------------- Locate Cameras -------------------------#

    # parameters
    #x0 = np.matrix([10000,5000,1390]) # real position of robot in mm
    x0 = np.matrix([650,940,190])
    n_cameras = [139,141]
    m = 4 #number of reference points
    img_dic = dict()
    cam_dic = dict()

    r_height = x0[0,2]

    for n in n_cameras:
        #---------------------------- Initialize --------------------------#
        cam = Camera(n)
        cam.read(cam_dir)
        # use newest file from this camera
        img=Image(n)
        img.read_ref(out_dir,"ref_",m)
        img.read_pos(out_dir,"pos_img")
        img.r_truth = x0
        #img.ref_real = img.augment(img.ref_real,np.zeros((4,1)))
        #z_ref = np.matrix([135,0,230,0]).T #in mm
        z_ref = ''
        img.ref_real = img.augment(img.ref_real,z_ref)
        #------------------------- Calibrate camera -----------------------#
        #flag = cv2.CV_P3P
        flag = cv2.CV_ITERATIVE
        #flag = cv2.CV_EPNP
        use_guess = 0
        cam.reposition(img.ref_real,img.ref_img,use_guess,flag)

        ######################### INDIVIDUAL METHODS #######################
        #------------------------- Check transform ------------------------#
        img.ref,err_img,__ = cam.check_imagepoints(img.augment(img.ref_real),img.ref_img)
        print("ind: ref point img match [px]",cam.n,"\n",err_img)
        #--------------------------- Get Robot 3D -------------------------#
        img.r,err2,err3 = get_leastsquares([cam],[img.augment(img.r_img)],
                                           'my',r_height,x0)
        print("ind: robot position [mm]",img.r.T,'2D',err2,'3D',err3)
        img_dic[n] = img
        cam_dic[n] = cam

    ############################### COMBINED METHODS #######################
    c139 = cam_dic[139]
    c141 = cam_dic[141]
    i139 = img_dic[139]
    i141 = img_dic[141]

    #------------------------------ Least squares -------------------------#
    p_ob1,err21,err31 = get_leastsquares([c139,c141],[i139.augment(i139.r_img),
                                   i141.augment(i141.r_img)],'hz',r_height,x0)
    p_ob2,err22,err32 = get_leastsquares([c139,c141],[i139.augment(i139.r_img),
                                   i141.augment(i141.r_img)],'my',r_height,x0)
    p_ob3,err23,err33 = get_leastsquares([c139,c141],[i139.augment(i139.r_img),
                                   i141.augment(i141.r_img)],'hz','',x0)
    p_ob4,err24,err34 = get_leastsquares([c139,c141],[i139.augment(i139.r_img),
                                   i141.augment(i141.r_img)],'my','',x0)
    print("lq: hz fixed height: robot position [mm]",p_ob1.T,'2D',err21,'3D',err31)
    #print("lq: my fixed height: robot position ",p_ob2.T,'2D',err22,'3D',err32)
    print("lq: hz free height: robot position [mm]",p_ob3.T,'2D',err23,'3D',err33)
    #print("lq: my free height: robot position ",p_ob4.T,'2D',err24,'3D',err34)
    #------------------------------ Triangulation -------------------------#
    # triangulate reference points
    opts_norm,err2r,err3r = triangulate(c139.Proj, c141.Proj,i139.ref_img.T,i141.ref_img.T,img.ref_real)
    print("triang: ref points error 2D [mm] \n",err2r)#,'3D',err3r)
    # triangulate robot point
    p_norm,err2p,err3p = triangulate(c139.Proj, c141.Proj,i139.r_img,i141.r_img,x0)
    print("triang: robot position [mm]",p_norm,"error 2D",err2p[0],"3D",err3p[0])
    print("\n real robot position [mm]",x0)

    '''H,mask  = cv2.findHomography(i1.ref_img,i2.ref_img)
    __,P,__ = cv2.estimateAffine3D(i1.augment(i1.ref_img),i2.augment(i2.ref_img))
    opts = cv.fromarray(opts)
    ipts = cv.fromarray(i1.ref_img)
    cam_C = cv.fromarray(cam.C)
    cam_dist = cv.fromarray(np.matrix(cam.dist))
    cam_r = cv.fromarray(cam.r)
    cam_t = cv.fromarray(cam.t)
    cv.FindExtrinsicCameraParams2(opts,ipts, cam_C,cam_dist,cam_r,cam_t)
    # Doesn't work for some reason:
    #retval,cam1,dist1,cam2,dist2,R,T,E,F = cv2.stereoCalibrate(opts,i1.ref_img,i2.ref_img,size)'''
