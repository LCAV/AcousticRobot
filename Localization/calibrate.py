-*- coding: utf-8 -*-
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

USAGE = '''
USAGE: calib.py [-o <output path>] [-c <camera path>] [-n <camera_number>]]
'''

def get_param():
    ''' returns parameters from command line '''
    global USAGE
    out_dir = ''
    cam_dir = ''
    n = 141
    try:
        opts,args = getopt.getopt(sys.argv[1:],"o:c:n:",
                                  ['save=', 'cam=','n='])
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
    if out_dir[-1]!="/":
        out_dir = out_dir+"/"
    if cam_dir[-1]!="/":
        cam_dir = cam_dir + "/"
    return out_dir,cam_dir,n
def get_calibpoints(img,pattern_size,counter,out_dir,n):
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
            cv2.imwrite('{0}/input{1}_{2}.jpg'.format(out_dir,n,counter),img)
            cv2.imwrite('{0}/output{1}_{2}.jpg'.format(out_dir,n,counter), out)
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
def get_leastsquares(cam1,cam2,p1,p2,method='hz',r_height=''):
    '''
    Solve least squares problem with or without fixed height
    Returns real position of robot based on observation with cam1 and cam2
    '''
    if method == 'hz':
        '''DLT Method of Hartley Zisserman, chapter 12.2 '''
        f1 = cam1.Proj[:1]
        f2 = cam1.Proj[1:2]
        f3 = cam1.Proj[2:3]
        g1 = cam2.Proj[:1]
        g2 = cam2.Proj[1:2]
        g3 = cam2.Proj[2:3]
        x1=p1[0,0]
        y1=p1[0,1]
        x2=p2[0,0]
        y2=p2[0,1]
        A_aug = np.vstack((x1*f3-f1,y1*f3-f2,x2*g3-g1,y2*g3-g2))
        if r_height:
            a1 = A_aug.T[:1]
            a2 = A_aug.T[1:2]
            a3 = A_aug.T[2:3]
            a4 = A_aug.T[3:4]
            A_aug2 = np.vstack((a1,a2)).T
            b_aug = -a3*r_height-a4
            x_hat = (A_aug2.T*A_aug2).I*A_aug2.T*b_aug.T
            control = A_aug*np.vstack((x_hat,r_height,1))
            return np.vstack((x_hat,r_height)),control,0
            # Algorithm page 590 if matrix A_aug is singular (not tested!
            u,w,v = svd(A_aug)
            b_prime = u.T*b_aug
            y = np.zeros((v.shape[1],1))
            for i in range(v.shape[1]):
                try:
                    y[i]=b_aug[0,i]/w[i]
                except:
                    y[i]=0
            x_hat = v[0]*y
        else:
            u,w,v = svd(A_aug)
            x_hat = v[-1]
            x_hat = x_hat/x_hat[3]
            return x_hat[:3],0,0
    else:
        ''' Own Method using algebraic transformations '''
        if r_height:
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
            return np.vstack((x_hat[2:4],r_height)),x_hat[0],x_hat[1]
        else:
            A_aug = np.vstack((cam1.Proj,cam2.Proj))
            b_aug = np.vstack((p1.T,p2.T))
            b_aug = b_aug.reshape((6,1))
            x_hat = (A_aug.T*A_aug).I*A_aug.T*b_aug
            x_hat = x_hat/x_hat[3]
            return x_hat[:3],0,0
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
def triangulate(P1,P2,i1,i2):
    p_test = cv2.triangulatePoints(P1,P2,i1,i2)
    p_norm = p_test/p_test[3]
    p_norm = p_norm.T[:,:3]
    return p_norm

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
        ''' Update R and Proj '''
        R,__ = cv2.Rodrigues(self.r)
        self.R = np.matrix(R)
        self.Proj = self.C*np.hstack((self.R,self.t))
    def read(self,cam_dir):
        ''' get camera parameters from file '''
        import numpy as np
        i=0
        rms = 0.0
        cam = np.zeros((3,3))
        dist = np.zeros((1,5))
        rvecs = []
        tvecs = []
        with open(cam_dir+"results"+str(self.n)+".txt","r") as f:
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
    def save(self,cam_dir):
        ''' save camera parameters in file '''
        with open(cam_dir+"results"+str(self.n)+".txt","w") as f:
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
    def get_checkpoints(self,out_dir):
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
                                                     counter,out_dir,self.n)
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
    def get_checkpoints_file(self,out_dir):
        '''
        gets checkboard points for the intrinsic camera  calibration. FROM FILES
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
                                                    counter,out_dir,self.n)
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
    def reposition(self,opts,ipts,guess=0,flag=cv2.CV_P3P):
        '''
        Redefines the position vectors r and t of the camera given a set of
        corresponding image and object points
        Iterative = 0, EPNP = 1, P3P = 2
        '''
        global px_size
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
        #rvec,tvec,__ = cv2.solvePnPRansac(opts,ipts,self.C,self.dist)
        __,rvec,tvec = cv2.solvePnP(opts,ipts,self.C,self.dist,
                                    self.r,self.t,guess,flag)
        self.r = np.matrix(rvec)
        self.t = np.matrix(tvec)
        self.update()
        print("Extrinsic calibration succeeded")
    def get_position(self,p_img,r_height):
        '''
        Returns the estimated position of the robot based on its
        image coordinates p_img and its fixed height r_height
        '''
        (r1,r2,r3) = self.R.T[:,:]
        A = np.hstack(([p_img.T,-self.C*r1.T,-self.C*r2.T]))
        b = self.C*(r3.T*r_height+self.t)
        x = A.I*b
        p_obj=(self.C*self.R).I*(p_img.T*x[0]-self.C*self.t)
        return p_obj.T
    def check_points(self,p_obj,p_img):
        '''
        Returns images of reference points (known) and the relative error matrix
        '''
        ### method 1
        #c = cam.C[:,2]
        #fx = cam.C[0,0]
        #fy = cam.C[1,1]
        # xc[n] = rmat_dic[n]*x0.T+t_dic[n]
        # xc_norm[n] = xc[n]/xc[n][2] # x', y'
        # xc_norm[n] = np.diag(np.matrix([fx,fy]).T*xc_norm[n][:2].T)+c[:2].T # u, v
        ### method 2
        # xc_norm[n],__ = cv2.projectPoints(np.array(x0,dtype=np.float32),rvec,tvec,cam,dist[0])
        ### best method
        dim = p_obj.shape[0]
        test_img = self.Proj*np.matrix(np.hstack((p_obj,np.ones((dim,1))))).T
        #print("\n",test_img)
        test_img = test_img/test_img[2,:]
        #print("\n",test_img)
        test_img=test_img[:2,:].T
        match_matrix=(p_img-test_img)/test_img
        ref = test_img
        return match_matrix,ref
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
    def __init__(self,n,ref_truth=0,r_truth=0,ref_img=0,ref=0,r_img=0,r=0):
        self.n = n
        self.ref_img = ref_img
        self.ref = ref
        self.r_img = r_img
        self.r = r
        self.r_truth = r_truth
        self.ref_truth = ref_truth
    def get_newest(self,dirname,fname):
        ''' Get newest file name '''
        f_newest = '0000000000'
        for f in os.listdir(dirname):
            if f.startswith(fname+str(self.n)):
                if int(f[-14:-4]) > int(f_newest[-14:-4]):
                    f_newest = f
        return dirname+f_newest
    def read_ref(self,dirname,fname):
        '''
        get reference point and robot img positions from newest file in directory
        'dirname'
        '''
        fname = self.get_newest(dirname,fname)
        #print("reading",fname)
        i = 0
        pts_img = np.zeros((4,2))
        M = np.zeros((3,3))
        pts_obj = np.zeros((4,2))
        with open(fname,"r") as f:
            c = csv.reader(f,delimiter = '\t')
            for line in c:
                if i >= 0 and i <4:
                    pts_img[i,:] = line
                elif i >= 4 and i < 7:
                    M[i-4,:] = line
                elif i >= 7 and i< 11:
                    pts_obj[i-7,:] = line
                i+=1
        img_points = pts_img.astype(np.float32)
        obj_points = pts_obj.astype(np.float32)
        self.ref_img = np.matrix(img_points)
        self.ref_truth = np.matrix(obj_points)
    def read_pos(self,dirname,fname):
        #fname = self.get_newest(dirname,fname)
        fname=dirname+fname+str(self.n)+".txt"
        with open(fname,"r") as f:
            c = csv.reader(f,delimiter = '\t')
            for line in c:
                p = [float(x) for x in line if x!='']
        # last value of p corresponds to last line
        self.r_img = np.array(p,dtype=np.float32)
    def augment(self,pt,z = ''):
        ''' Add third dimension to points (ones if not specified) '''
        if z=='':
            try:
                return np.matrix(np.hstack((pt,1)))
            except:
                dim = pt.shape[0]
                return np.matrix(np.hstack((pt,np.ones((dim,1))))).astype(np.float32)
        else:
            return np.matrix(np.hstack((pt,z.reshape(4,1)))).astype(np.float32)

global px_size
px_size = 1.4*10**-3 # mm per pixel

if __name__ == '__main__':
    import getopt

    #------------------------- Get parameters -------------------------#
    out_dir,cam_dir,n = get_param()
    #------------------------ Calibrate Camera ------------------------#

    # make new calibration
    #camera = Camera(n)
    #img_points,obj_points,size = camera.get_checkpoints(out_dir)
    #root mean square (RMS) re-projection error (good between 0.1 and 1)
    #camera.calibrate(obj_points,img_points,size)
    #camera.save(out_dir)

    #-------------------------- Undistort   ---------------------------#

    # load calibration
    #camera2 = Camera(n)
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

    img_dic = dict()
    cam_dic = dict()

    r_height = x0[0,2]
    r_height_px = r_height/px_size # robot height in pixels

    for n in n_cameras:
        #---------------------------- Initialize --------------------------#
        cam = Camera(n)
        cam.read(cam_dir)
        # use newest file from this camera
        img=Image(n)
        img.read_ref(out_dir,"ref_")
        img.read_pos(out_dir,"pos_img")
        img.r_truth = x0
        #img.ref_truth = img.augment(img.ref_truth,np.zeros((4,1)))
        z_ref = np.matrix([135,0,230,0]).T #in mm
        img.ref_truth = img.augment(img.ref_truth,z_ref)
        #------------------------- Calibrate camera -----------------------#
        flag = cv2.CV_P3P
        #flag = cv2.CV_ITERATIVE
        #flag = cv2.CV_EPNP
        use_guess = 0
        cam.reposition(img.ref_truth,img.ref_img,use_guess,flag)

        ######################### INDIVIDUAL METHODS #######################
        #------------------------- Check transform ------------------------#
        match_matrix,img.ref = cam.check_points(img.ref_truth,img.ref_img)
        print("ind: ref point match",cam.n,"\n",match_matrix)
        #--------------------------- Get Robot 3D -------------------------#
        img.r = cam.get_position(img.augment(img.r_img),r_height)
        print("ind: robot position",img.r)
        img_dic[n] = img
        cam_dic[n] = cam

    ############################### COMBINED METHODS #######################
    c139 = cam_dic[139]
    c141 = cam_dic[141]
    i139 = img_dic[139]
    i141 = img_dic[141]

    #------------------------------ Least squares -------------------------#
    p_obj,__,__ = get_leastsquares(c139,c141,i139.augment(i139.r_img),
                                   i141.augment(i141.r_img),'hz',r_height)
    p_tst,__,__ = get_leastsquares(c139,c141,i139.augment(i139.r_img),
                                   i141.augment(i141.r_img),'my',r_height)
    print("lq fixed height: robot position ",p_obj.T)
    print("lq free height:: robot position ",p_tst.T)
    #------------------------------ Triangulation -------------------------#
    # triangulate reference points
    opts_norm = triangulate(c139.Proj, c141.Proj,i139.ref_img.T,i141.ref_img.T)
    opts_theo = img.ref_truth #same for both cameras
    print("triang: ref point match\n",((opts_theo-opts_norm)/opts_norm))

    # triangulate robot point
    p_norm = triangulate(c139.Proj, c141.Proj,i139.r_img,i141.r_img)
    print("triang: robot position",p_norm)
    print("\n real robot position:",x0)

    H,mask  = cv2.findHomography(i1.ref_img,i2.ref_img)
    __,P,__ = cv2.estimateAffine3D(i1.augment(i1.ref_img),i2.augment(i2.ref_img))
    opts = cv.fromarray(opts)
    ipts = cv.fromarray(i1.ref_img)
    cam_C = cv.fromarray(cam.C)
    cam_dist = cv.fromarray(np.matrix(cam.dist))
    cam_r = cv.fromarray(cam.r)
    cam_t = cv.fromarray(cam.t)
    cv.FindExtrinsicCameraParams2(opts,ipts, cam_C,cam_dist,cam_r,cam_t)
    # Doesn't work for some reason:
    #retval,cam1,dist1,cam2,dist2,R,T,E,F = cv2.stereoCalibrate(opts,i1.ref_img,i2.ref_img,size)
