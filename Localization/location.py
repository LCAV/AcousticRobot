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
    out_dir = 'calib/test_tuesday/'
    n_cameras = [139,141]
    #p_real = np.matrix([750,790,190])
    p_real = np.matrix([650,940,190])
    r_height = p_real[0,2] #height of robot in mm
    #ref_z = np.array([135,0,230,0]) #height of reference points in mm
    ref_z = '' #height automatically set to 0
    flag = 0
#--------------------------- 0. Intrinsic Calibration   -----------------------#

    n = raw_input("Intrinsic Calibration: Enter camera number or 'q' to skip: ")
    if n!='q':
        n=int(n)
        cam = calib.Camera(n)
        img_points,obj_points,size = cam.get_checkpoints(out_dir)
        cam.calibrate(obj_points,img_points,size)
        cam.save(cam_dir)
#--------------------------- 3. Extrinsic Calibration   -----------------------#
#--------------------------- 3.a Get Image Points       -----------------------#
    n = ''
    while True:
        n = raw_input("Extrinsic Calibration: Enter camera number or 'q' to skip: ")
        if n != 'q':
            plt.close('all')
            n = int(n)
            cam = calib.Camera(n)
            cam.read(cam_dir)
            img = get.get_image(n)

            #-- undistort image --#
            img = cam.undistort(img)

            #-- get points --#
            col_min = np.array([175,100,0],dtype=np.uint8)
            col_max = np.array([20,250,255],dtype=np.uint8)
            r = 30
            n_pts = 4
            t = 100
            img_org,circ_org,pts_img,th_org = persp.imagepoints(img,r,n_pts,t,
                                                                col_min,col_max)
        else:
            break
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
        img.ref_truth = img.augment(img.ref_truth,ref_z)
        cam.reposition(img.ref_truth,img.ref_img)
#--------------------------- 4. Localization            -----------------------#
#--------------------------- 4.a Get Image Points       -----------------------#
    choice = raw_input("Do you want to locate the robot? (y/n) ")
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
            img.ref_truth = img.augment(img.ref_truth,ref_z)
            cam.reposition(img.ref_truth[:3],img.ref_img[:3],0,flag)
            img.r = cam.get_position(img.augment(img.r_img),r_height)
            img_dic[n] = img
            cam_dic[n] = cam

        # TODO: For all permutations (when more than 1 camera)
        c1 = cam_dic[139]
        c2 = cam_dic[141]
        i1 = img_dic[139]
        i2 = img_dic[141]
        p_lq,__,__ = calib.get_leastsquares(c1,c2,i1.augment(i1.r_img),
                                           i2.augment(i2.r_img),'hz',r_height)
        p_lq = p_lq.reshape((1,3))
        d_lq = np.sqrt(np.sum(np.power(p_real-p_lq,2)))
        d_lq2 = np.sqrt(np.sum(np.power(p_real[0,:2]-p_lq[0,:2],2)))
        print("Least squares: ",p_lq,"error 3D: ",d_lq,"/ 2D: ",d_lq2)
        p_tr = calib.triangulate(c1.Proj, c2.Proj,i1.r_img,i2.r_img)
        p_tr = np.matrix(p_tr.reshape((1,3)))
        d_tr = np.sqrt(np.sum(np.power(p_real-p_tr,2)))
        d_tr2 = np.sqrt(np.sum(np.power(p_real[0,:2]-p_tr[0,:2],2)))
        p_lq = p_lq.reshape((1,3))
        print("Triangulation: ",p_tr,"error 3D: ",d_tr,"/ 2D: ",d_tr2)
        print("Real position: ",p_real)
        match_matrix1,i1.ref = c1.check_points(i1.ref_truth,i1.ref_img)
        match_matrix2,i2.ref = c2.check_points(i2.ref_truth,i2.ref_img)
        #print("Ref point match",c1.n,"\n",match_matrix1)
        #print("Ref point match",c2.n,"\n",match_matrix2)
        choice = raw_input("Do you want to locate the robot? (y/n) ")

#--------------------------- 4.c Make Robot Move        -----------------------#
