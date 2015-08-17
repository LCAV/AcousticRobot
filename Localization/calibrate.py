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
# built-in modules
import os

USAGE = '''
USAGE: calib.py [-o <output path>] [-s <square size in mm>] [-n <camera_number>]]
'''

global save_dir
global n
global square_size
def get_param():
    ''' returns parameters from command line '''
    global USAGE
    save_dir = ''
    square_size = 50 #in mm
    n = 141
    try:
        opts,args = getopt.getopt(sys.argv[1:],"o:s:n:",
                                  ['save=', 'square_size=','n='])
    except getopt.GetoptError:
        print(USAGE)
        sys.exit(2)
    for opt, arg in opts:
        if opt in ('--save',"-o"):
            save_dir = str(arg)
        if opt in ("--square_size", "-s"):
            square_size = float(arg)
        if opt in ('--n',"-n"):
            n = int(arg)
    return save_dir, square_size,n

def save_camera(rms,cam,dist,rvecs,tvecs):
    ''' save camera parameters in file '''
    with open(save_dir+"/results"+str(n)+".txt","w") as f:
        f.write(str(rms)+'\n')
        for e in cam:
            f.write("{0:10.4f}\t{1:10.4f}\t{2:10.4f}\n".format(e[0],e[1],e[2]))
        for e in dist[0]:
            f.write("{:10.4f}\t".format(e))
        f.write("\n")
        for j in range(len(rvecs)):
            for i in range(3):
                f.write("{:10.4f}\t".format(rvecs[j][i][0]))
            f.write("\n")
        for j in range(len(tvecs)):
            for i in range(3):
                f.write("{:10.4f}\t".format(tvecs[j][i][0]))
            f.write("\n")

def read_camera():
    ''' get camera parameters from file '''
    global save_dir, n
    import numpy as np
    i=0
    rms = 0.0
    cam = np.zeros((3,3))
    dist = np.zeros((1,5))
    rvecs = []
    tvecs = []
    with open(save_dir+"/results"+str(n)+".txt","r") as f:
        print("reading",n,"...")
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
    return rms,cam,[np.array(dist)],np.array(rvecs),np.array(tvecs)

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

def read_positions(newest,dirname='pos3D/',fname=''):
    if newest:
        f_newest = '0000000000'
        for f in os.listdir(dirname):
            if f.startswith("pos_"+str(n)):
                if int(f[-14:-4]) > int(f_newest[-14:-4]):
                    f_newest = f
        fname = dirname+f_newest
    ''' get reference point and robot positions from file '''
    i = 0
    pts_img = np.zeros((4,2))
    M = np.zeros((3,3))
    pts_obj = np.zeros((4,2))
    with open(fname,"r") as f:
        c = csv.reader(f,delimiter = '\t')
        for line in c:
            if i == 0:
                p = [float(x) for x in line if x!='']
            elif i > 0 and i <=4:
                pts_img[i-1,:] = line
            elif i > 4 and i <= 7:
                M[i-5,:] = line
            elif i > 7 and i<= 11:
                pts_obj[i-8,:] = line
            i+=1
    obj_points = np.hstack((pts_obj,np.ones((4,1))))
    #obj_points = np.hstack((pts_obj,np.matrix([13.5,0,23.0,0]).T))#in cm
    img_points = pts_img

    obj_points = obj_points.astype(np.float32)
    img_points = img_points.astype(np.float32)
    return obj_points, img_points, p

def get_calibpoints(img,pattern_size, counter):
    '''
    returns pairs of object- and image points (chess board corners)
    for camera calibration
    '''
    global save_dir,n,square_size
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
        if save_dir:
            print("saving...")
            out = img.copy()
            cv2.drawChessboardCorners(out, pattern_size, corners, found)
            ax.imshow(out),plt.show(block=False)
            cv2.imwrite('{0}/input{1}_{2}.jpg'.format(save_dir,n,counter),img)
            cv2.imwrite('{0}/output{1}_{2}.jpg'.format(save_dir,n,counter), out)
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

def undo_fisheye(img,cam,dist):
    # intrinsic parameters
    fx = cam[0,0]
    fy = cam[1,1]
    cx = cam[0,2]
    cy = cam[1,2]
    # radial distortion coefficients
    k1,k2 = dist[0][0:2]
    # †angential distortion coefficients
    p1,p2 = dist[0][2:4]
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

def get_intrinsic():
    ''' intrinsic (and extrinsic) camera  calibration using checkboard '''
    choice = "y"
    counter = 1
    obj_points = []
    img_points = []
    pattern_size = (8, 5)
    while True:
        try:
            # Collect Data
            if choice == "y":
                img = get_image.get_image(n)
                h, w = img.shape[:2]
                img_pt, obj_pt = get_calibpoints(img,pattern_size,counter)
                if not img_pt == None:
                    img_points.append(img_pt)
                    obj_points.append(obj_pt)
                counter += 1
                choice = raw_input("Do you want to take another image? (y/n)")
            elif choice == "n":
            # Calibrate Camera
                return img_points, obj_points, (w,h)
        except KeyboardInterrupt:
            print("program terminated by user")
            sys.exit(1)

class Camera:
    def __init__(self,n,C,dist,r,t):
        # “raw parameters"
        self.n = n
        self.C = np.matrix(C)
        self.dist = np.matrix(dist)
        self.r = np.matrix(r)
        self.t = np.matrix(t)
        # "combined parameters"
        R,__ = cv2.Rodrigues(self.r)
        self.R = np.matrix(R)
        self.Proj = self.C*np.hstack((self.R,self.t))
    def reposition(self,opts,ipts,guess,flag):
        '''
        Redefines the position vectors r and t of the camera given a set of
        corresponding image and object points
        '''
        global px_size
        if flag == cv2.CV_P3P or flag == cv2.CV_EPNP:
            ipts = np.ascontiguousarray(ipts[:,:2]).reshape((4,1,2))
        if guess:
            tguess = {139:np.array([650,120]),
                       141:np.array([450,50])}
            r_guess,h_guess = get_rvguess(self.r,self.t*px_size)
            t_guess=np.hstack([tguess[self.n],h_guess])
            self.t = np.matrix(t_guess)
            self.r = np.matrix(r_guess)
        #rvec,tvec,__ = cv2.solvePnPRansac(opts,ipts,self.C,self.dist)
        __,rvec,tvec = cv2.solvePnP(opts,ipts,self.C,self.dist,
                                    self.r,self.t,guess,flag)
        self.r = rvec
        self.t = tvec
        R,__ = cv2.Rodrigues(self.r)
        self.R = np.matrix(R)
        self.Proj = self.C*np.hstack((self.R,self.t))
    def get_position(self,p_img,r_height):
        '''
        Returns the estimated position of the robot based on its
        image coordinates p_img and its fixed height r_height
        '''
        (r1,r2,r3) = self.R.T[:,:]
        A[n] = np.hstack(([p_img.T,-self.C*r1.T,-self.C*r2.T]))
        b[n] = camera.C*(r3.T*r_height+self.t)
        x = A[n].I*b[n]
        p_obj=(self.C*camera.R).I*(p_img.T*x[0]-self.C*self.t)
        return p_obj.T
    def check_points(self,p_obj,p_img):
        '''
        Returns images of reference points (known) and the relative error matrix
        '''
        dim = p_obj.shape[0]
        test_img = self.Proj*np.matrix(np.hstack((p_obj,np.ones((dim,1))))).T
        #print("\n",test_img)
        test_img = test_img/test_img[2,:]
        #print("\n",test_img)
        test_img=test_img[:2,:].T
        match_matrix=(p_img-test_img)/test_img
        return match_matrix, test_img

def get_leastsquares(cam1,cam2,p1,p2):
    '''
    Returns real position of robot based on observation with cam1 and cam2
    '''
    u1 = p1.T
    u2 = p2.T
    U = np.array([u1,np.zeros((3,1)),np.zeros((3,1)),u2])
    U = U.reshape((2,6,1))
    U = U[:,:,0].T
    (r1,r2,r3) = cam1.R.T[:,:]
    A1 = np.hstack(([-cam1.C*r1.T,-cam1.C*r2.T]))
    b1 = cam1.C*(r3.T*r_height+cam1.t)
    (r1,r2,r3) = cam2.R.T[:,:]
    A2 = np.hstack(([-cam2.C*r1.T,-cam2.C*r2.T]))
    b2 = cam2.C*(r3.T*r_height+cam2.t)
    A_aug = np.hstack((U,np.vstack((A1,A2))))
    b_aug = np.vstack((b1,b2))
    x_hat = (A_aug.T*A_aug).I*A_aug.T*b_aug
    return x_hat[2:4],x_hat[0],x_hat[1]
global px_size

if __name__ == '__main__':
    import sys
    import getopt
    from glob import glob
    import get_image

    px_size = 1.4*10**-3 # mm per pixel
    #------------------------- Get parameters -------------------------#

    save_dir,square_size,n = get_param()
    square_size = square_size/px_size # †his doesn't matter for camera calibration!!
    #------------------------ Calibrate Camera ------------------------#

    # make new calibration
    #img_points,obj_points,size = get_intrinsic()
    #rms,cam,dist,rv,tv = cv2.calibrateCamera(obj_points,img_points,size, None, None)
    #save_camera(rms,cam,dist,rv,tv)

    #-------------------------- Undo Fisheye --------------------------#

    # load calibration
    #__,cam,dist,rv,tv = read_camera()
    #for f in os.lsitdir(save_dir):
    #    if f.startswith("input"):
    #        img_in=cv2.imread(save_dir+"/"+f,cv2.IMREAD_COLOR)
    #        undone = undo_fisheye(img_in,cam,dist)
    #        cv2.imwrite(save_dir+"/"+f.replace("input","undone"),undone)

    #------------------------- Locate Cameras -------------------------#

    # parameters
    #x0 = np.matrix([10000,5000,1390]) # real position of robot in mm
    x0 = np.matrix([650,940,190])

    n_cameras = [139,141]

    xc = dict()
    xc_norm = dict()
    ipts_dic = dict()
    r_img_dic = dict()
    r_obj_dic = dict()
    A = dict()
    b = dict()

    r_height = x0[0,2]

    r_height_px = r_height/px_size # robot height in pixels

    camera_dic = dict()
    for i in n_cameras:
        n = i
        __,cam,dist,rv,tv = read_camera()
        camera_dic[n] = Camera(n,cam,dist,rv.T,tv.T)

    for (count,camera) in iteritems(camera_dic):
        n = camera.n
        # use newest file from this camera
        newest = 1
        opts,ipts,p = read_positions(newest,'pos/')
        opts = opts*10
        opts = opts.astype(np.float32) #object points in mm
        ipts_dic[n] = np.matrix(ipts)
        r_img_dic[n]=np.matrix(np.hstack((p,1)))
        flag = cv2.CV_P3P
        #flag = cv2.CV_ITERATIVE
        #flag = cv2.CV_EPNP
        use_guess = 0
        camera.reposition(opts,ipts_dic[n],use_guess,flag)
        #-------------------------- Check transform -----------------------#
        # check with robot position
        ### method 1
        #c = camera.C[:,2]
        #fx = camera.C[0,0]
        #fy = camera.C[1,1]
        # xc[n] = rmat_dic[n]*x0.T+t_dic[n]
        # xc_norm[n] = xc[n]/xc[n][2] # x', y'
        # xc_norm[n] = np.diag(np.matrix([fx,fy]).T*xc_norm[n][:2].T)+c[:2].T # u, v
        ### method 2
        # xc_norm[n],__ = cv2.projectPoints(np.array(x0,dtype=np.float32),rvec,tvec,cam,dist[0])
        ### best method
        match_matrix, xc_norm[n] = camera.check_points(opts,ipts_dic[n])
        print("ind: ref point match",camera.n,"\n",match_matrix)

        #--------------------------- Get Robot 3D -------------------------#
        r_obj_dic[n] = camera.get_position(r_img_dic[n],r_height)
        print("ind: robot position",r_obj_dic[n])

    # lest squares for two cameras


    c139 = camera_dic[139]
    c141 = camera_dic[141]

    p_obj,s139,s141 = get_leastsquares(c139,c141,r_img_dic[139],r_img_dic[141])
    print("lq: robot position ",p_obj)

    # triangulate reference points
    opts_test = cv2.triangulatePoints(c139.Proj, c141.Proj,np.array(ipts_dic[139]).T,np.array(ipts_dic[141]).T)
    opts_norm = opts_test/opts_test[3]
    opts_norm = opts_norm.T[:,:3]
    opts_theo = opts #same for both cameras
    print("triang: ref point match\n",((opts_theo-opts_norm)/opts_norm))

    # triangulate robot point
    p_test = cv2.triangulatePoints(c139.Proj, c141.Proj,r_img_dic[139][0,:2].T,
                                   r_img_dic[141][0,:2].T)
    p_norm2 = p_test/p_test[3]
    p_norm2 = p_norm2.T[:,:3]
    print("triang: robot position",p_norm2)
    print("\n real robot position:",x0)
