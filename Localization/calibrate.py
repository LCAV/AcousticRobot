#-*- coding: utf-8 -*-
#!/usr/bin/env python
from __future__ import division
from __future__ import print_function
import numpy as np
import cv2
import cv2.cv as cv
import matplotlib.pyplot as plt
import csv
# built-in modules
import os

USAGE = '''
USAGE: calib.py [--save <output path>] [--square_size <square size>] [--n <camera_number>]]
'''

save_dir = ''
n = 139

def get_param():
    ''' returns parameters from command line '''
    global USAGE
    save_dir = ''
    square_size = 150
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
    global save_dir, n
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

def read_positions(fname):
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
    obj_points = np.hstack((pts_obj,np.zeros((4,1))))
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

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    pattern_points = np.zeros( (np.prod(pattern_size), 3), np.float32 )
    pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points *= square_size

    h, w = 0, 0
    found, corners = cv2.findChessboardCorners(gray, pattern_size)
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
                rms,cam,dist,rv,tv = cv2.calibrateCamera(obj_points, img_points,
                                                         (w,h), None, None)
                return 1
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


if __name__ == '__main__':
    import sys
    import getopt
    from glob import glob
    import get_image

    #------------------------- Get parameters -------------------------#

    save_dir,square_size,n = get_param()

    #------------------------ Calibrate Camera ------------------------#

    # make new calibration
    # rms,cam,dist,rv,tv = get_intrinsic()
    # save_camera(rms,cam,dist,rv,tv)

    # load calibration
    #__,cam,dist,rv,tv = read_camera()

    #-------------------------- Undo Fisheye --------------------------#

    #for f in os.lsitdir(save_dir):
    #    if f.startswith("input"):
    #        img_in=cv2.imread(save_dir+"/"+f,cv2.IMREAD_COLOR)
    #        undone = undo_fisheye(img_in,cam,dist)
    #        cv2.imwrite(save_dir+"/"+f.replace("input","undone"),undone)

    #------------------------- Locate Cameras -------------------------#

    # parameters
    px_size = 1.4*10**-3 # mm per pixel
    x0 = np.matrix([10000,5000,0])
    n_cameras = [139,141]

    xc = dict()
    xc_norm = dict()
    ipts_dic = dict()
    opts_dic = dict()
    cam_mat_dic = dict()
    rmat_dic = dict()
    t_dic = dict()
    p_img_dic = dict()
    p_obj_dic = dict()
    r_height = 1000 # robot height in mm
    r_height_px = r_height/px_size # robot height in pixels

    for n in n_cameras:
        __,cam,dist,rv,tv = read_camera()
        camera = Camera(n,cam,dist,rv.T,tv.T)
        # use newest file from this camera
        f_newest = '0000000000'
        for f in os.listdir('pos/'):
            if f.startswith("pos_"+str(n)):
                if int(f[-10:]) > int(f_newest[-10:]):
                    f_newest = f
        fname = 'pos/'+f_newest
        opts,ipts,p = read_positions(fname)
        #cam = cam*px_size # camera matrix in mm
        #ipts = px_size*ipts # img points in mm
        ipts_dic[n] = ipts
        opts_dic[n] = opts*10 #object points in mm
        __,rvec,tvec = cv2.solvePnP(opts_dic[n],ipts_dic[n],cam,dist[0],rv,tv,0,cv2.CV_ITERATIVE)

        #-------------------------- Check transform -----------------------#
        rmat, j = cv2.Rodrigues(rvec)
        rmat_dic[n] = np.matrix(rmat)
        cam_mat_dic[n] = np.matrix(cam)
        t_dic[n]= np.matrix(tvec)

        # check with robot position
        c = cam_mat_dic[n][:,2]
        fx = cam_mat_dic[n][0,0]
        fy = cam_mat_dic[n][1,1]
        ### method 1
        # xc[n] = rmat_dic[n]*x0.T+t_dic[n]
        # xc_norm[n] = xc[n]/xc[n][2] # x', y'
        # xc_norm[n] = np.diag(np.matrix([fx,fy]).T*xc_norm[n][:2].T)+c[:2].T # u, v
        ### method 2
        # xc_norm[n],__ = cv2.projectPoints(np.array(x0,dtype=np.float32),rvec,tvec,cam,dist[0])
        ### best method
        xc[n] = cam_mat_dic[n]*np.hstack((rmat_dic[n],t_dic[n]))*np.vstack((x0.T,1))
        s = xc[n][2]
        xc_norm[n] = xc[n]/s
        print("robot, test:",np.array(xc_norm[n])[0:2].T[0],"real: check image")
        # check with reference point
        p1 = opts_dic[n][1]
        #xc_norm[n],__ = cv2.projectPoints(np.array([p1]),rvec,tvec,cam,dist[0])
        xc[n] = cam_mat_dic[n]*np.hstack((rmat_dic[n],t_dic[n]))*np.matrix(np.hstack((p1,1))).T
        xc_norm[n] = xc[n]/xc[n][2]
        print("reference, test:",np.array(xc_norm[n])[0:2].T[0],"real:",ipts_dic[n][1])

        #--------------------------- Get Robot 3D -------------------------#
        (r1,r2,r3) = rmat_dic[n].T[:,:]
        ### method 1 doesn't work cause not everything in one plane
        H = cam_mat_dic[n] * np.hstack((r1.T,r2.T,t_dic[n]))
        H = H/t_dic[n][2]
        p_img_dic[n] = np.matrix(np.hstack((p,1)))
        p_obj_dic[n] = H*p_img_dic[n].T
        ### method 2
        A = np.hstack(([p_img_dic[n].T,-cam_mat_dic[n]*r1.T,-cam_mat_dic[n]*r2.T]))
        b = cam_mat_dic[n]*(r3.T*r_height+t_dic[n])
        p_obj_dic[n] = A.I*b
        control=(cam_mat_dic[n]*rmat_dic[n]).I*(p_img_dic[n].T*p_obj_dic[n][0]
                                                -cam_mat_dic[n]*t_dic[n])
        t_dic[n]= np.matrix(tvec)
