# -*- coding: utf-8 -*-
from __future__ import division, print_function
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv
import getopt
import sys
import cv2

def get_parameters():
    fname = 'pos/pos_139_1439194203'
    try:
        opts,args = getopt.getopt(sys.argv[1:],"i:")
    except getopt.GetoptError:
        pritn("usage file.py -i <inputfile>")
        sys.exit(2)
    for opt,arg in opts:
        if opt == "-i":
            fname = arg
    return fname

def get_positions(fname):
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
    return np.array(p),np.array(pts_img),np.matrix(M),np.array(pts_obj)

def visualize_points(pts_obj,pts_img,M):
    ''' creates 3D graphs of image- and object points '''

    # fix real points to constant height
    pts_obj_3D = np.hstack((pts_obj,np.ones((4,1))))
    pts_obj_3D = np.matrix(pts_obj_3D)
    pts_img_3D = M.I*pts_obj_3D.T
    test_img = (pts_img_3D/pts_img_3D[2,:]).T
    pts_img_3D = pts_img_3D.T
    pts_obj_3D = pts_obj_3D.T

    # fix image points to constant height
    # pts_img_3D = np.hstack((pts_img,np.ones((4,1))))
    # pts_img_3D = np.matrix(pts_img_3D)
    # pts_obj_3D = M*pts_img_3D.T
    #test_real = (pts_obj_3D/pts_obj_3D[2,:]).T

    p_old = pts_img_3D.T[:,3]
    p_prime_old = pts_obj_3D[:,3]

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    colors = ['r','b','g','m']
    for i in range(4):
        p = pts_img_3D.T[:,i]
        p_prime = pts_obj_3D[:,i]

        t = np.linspace(-1,2,100)
        s = np.linspace(-1,2,100)
        # projection line
        d = (p - p_prime)*t + p_prime

        # Camera center calculation
        d_old = (p_old - p_prime_old)*s + p_prime_old

        # visualization
        x = np.array(d[0,:])[0]
        y = np.array(d[1,:])[0]
        z = np.array(d[2,:])[0]
        ax.plot(x,y,z,label='d'+str(i),color=colors[i])

        # image lines
        v = np.linspace(0,1,100)
        d2 = p_old + (p - p_old)*v
        x2 = np.array(d2[0,:])[0]
        y2 = np.array(d2[1,:])[0]
        z2 = np.array(d2[2,:])[0]
        # real lines
        d3  = p_prime_old + (p_prime - p_prime_old)*v
        x3 = np.array(d3[0,:])[0]
        y3 = np.array(d3[1,:])[0]
        z3 = np.array(d3[2,:])[0]
        if i < 3:
            ax.plot(x2,y2,z2,color='y')
            ax.plot(x3,y3,z3,color='k')
        else:
            ax.plot(x2,y2,z2,color='y',label='image')
            ax.plot(x3,y3,z3,color='k',label='real')
            ax.legend()
        p_old = p
        p_prime_old = p_prime

    plt.show(block=False)

if __name__ = '__main__':
    fname = get_parameters()
    p,pts_img,M,pts_obj = get_positions(fname)
    visualize_points(pts_obj,pts_img,M)

    camera_matrix[1,1]= 2200.0
    camera_matrix[2,2]=1.0
    camera_matrix[0,2]=750.0
    camera_matrix[1,2]=750.0

    dist_coefs = np.zeros(4)

    obj_points = np.hstack((pts_obj,np.zeros((4,1))))
    img_points = pts_img

    obj_points = obj_points.astype(np.float32)
    img_points = img_points.astype(np.float32)

    h = 1944
    w = 2592
    size=(w,h)
    sys.exit(1)
    retval,cam,dist,rvecs,tvecs = cv2.calibrateCamera([obj_points], [img_points],
                                                    size, camera_matrix, dist_coefs,
                                                    flags=cv2.CALIB_USE_INTRINSIC_GUESS)
