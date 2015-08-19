#-*- coding: utf-8 -*-
from __future__ import division
from __future__ import print_function

import calibrate as calib
import perspective as persp
import get_image as get
import marker_calibration as mark
import numpy as np
import matplotlib.pyplot as plt
import time

if __name__ == '__main__':
#---------------------------       Initialization       -----------------------#
    cam_dir = 'calib/'
    out_dir = 'calib/test_wednesday/'
    n_cameras = [139,141]
    r_height = 190 #height of robot in mm
#--------------------------- 0. Intrinsic Calibration   -----------------------#

    n = raw_input("Intrinsic Calibration: Enter camera number or 'q' to skip: ")
    if n!='q':
        n = int(n)
        cam = calib.Camera(n)
        img_points,obj_points,size = cam.get_checkpoints(out_dir)
        cam.calibrate(obj_points,img_points,size)
        cam.save(cam_dir)
#--------------------------- 3. Extrinsic Calibration   -----------------------#
#--------------------------- 3.a Get Image Points       -----------------------#

    n = raw_input("Extrinsic Calibration: Enter camera number or 'q' to skip: ")
    if n!='q':
        plt.close('all')
        n = int(n)
        cam = calib.Camera(n)
        cam.read(cam_dir)
        img = get.get_image(n)

        #-- undistort image --#
        #img = cam.undistort(img)

        #-- get points --#
        col_min = np.array([175,100,0],dtype=np.uint8)
        col_max = np.array([20,250,255],dtype=np.uint8)
        r = 30
        n_pts = 4
        t = 100
        img_org,circ_org,pts_img,th_org = persp.imagepoints(img,r,n_pts,t,
                                                            col_min,col_max)
#--------------------------- 3.b Get Object Points      -----------------------#
        #-- get points --#
        pts_obj, margin = persp.objectpoints()
        img_test,pts_obj,size = persp.format_points(pts_obj,margin)
        img_flat,M = persp.geometric_transformation(img,pts_obj,pts_img,size)

        #-- save results --#
        current_time = str(int(time.mktime(time.gmtime())))
        name='ref_' +str(n)+'_'+current_time+str('.txt')
        persp.write_ref(out_dir,name,pts_img,M,pts_obj)
#--------------------------- 3.c Calibrate              -----------------------#
        img = calib.Image(n)
        img.read_ref(out_dir,"ref_")
        # add z component to ref_truth
        img.ref_truth = img.augment(img.ref_truth)
        cam.reposition(img.ref_truth,img.ref_img)
#--------------------------- 4. Localization            -----------------------#
    choice = raw_input("Do you want to locate the robot? (y/n) ")
#--------------------------- 4.a Get Image Points       -----------------------#
    while choice == "y":
        for n in n_cameras:
            cam = calib.Camera(n)
            cam.read(cam_dir)
            img = get.get_image(n)

            #-- undistort image --#
            img = cam.undistort(img)

            #-- get point --#
            col_min = np.array([150,100,0],dtype=np.uint8)
            col_max = np.array([20,250,255],dtype=np.uint8)
            r = 40
            n_pts = 1
            t = 100
            img_red,circ_red,p,th_red = persp.imagepoints(img,r,n_pts,t,
                                                          col_min,col_max)
            #-- save resutls --#
            name='pos_img'+str(n)+'.txt'
            persp.write_pos(out_dir,name,p)

#--------------------------- 4.b Calculate Object Point -----------------------#
        img_dic = dict()
        cam_dic = dict()

        for n in n_cameras:
            cam = calib.Camera(n)
            cam.read(cam_dir)
            img = calib.Image(n)
            img.read_ref(out_dir,"ref_")
            img.read_pos(out_dir,"pos_img")
            img.ref_truth = img.augment(img.ref_truth)
            cam.reposition(img.ref_truth,img.ref_img)
            img.r = cam.get_position(img.augment(img.r_img),r_height)
            img_dic[n] = img
            cam_dic[n] = cam

        # TODO: For all permutations (when more than 1 camera)
        c1 = cam_dic[139]
        c2 = cam_dic[141]
        i1 = img_dic[139]
        i2 = img_dic[141]
        p_lq,__,__ = calib.get_leastsquares(c1,c2,i1.augment(i1.r_img),
                                           i2.augment(i2.r_img),r_height)
        print("Least squares: ",p_lq.T)
        p_tr = calib.triangulate(c1.Proj, c2.Proj,i1.r_img,i2.r_img)
        print("Triangulation: ",p_tr)
#--------------------------- 4.c Make Robot Move        -----------------------#
