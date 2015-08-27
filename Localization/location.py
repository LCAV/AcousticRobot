#-*- coding: utf-8 -*-
#!/usr/bin/env python
from __future__ import division
from __future__ import print_function

import calibrate as calib
import perspective as persp
import get_image as get
import marker_calibration as mark
import numpy as np
import matplotlib.pyplot as plt
import time
USAGE = '''
USAGE: locate.py -o <output path> -n <number of points>
'''
def get_param():
    ''' returns parameters from command line '''
    global USAGE
    out_dir = ''
    m = 0
    try:
        opts,args = getopt.getopt(sys.argv[1:],"o:m:",['output=','m='])
    except getopt.GetoptError:
        print(USAGE)
        sys.exit(2)
    for opt, arg in opts:
        if opt in ('--output',"-o"):
            out_dir = str(arg)
        if opt in ('--m',"-m"):
                m = int(arg)
    if out_dir[-1]!="/":
        out_dir = out_dir+"/"
    return out_dir,m

if __name__ == '__main__':
    import sys
    import getopt
#---------------------------       Initialization       -----------------------#
    cam_dir = 'calib/'
    out_dir,n_pts = get_param() # number of reference points
    n_cameras = [139,141]
    #p_real = np.matrix([750,790,190])
    p_real = np.matrix([650,940,170])
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
            t = 100
            img_org,circ_org,pts_img,th_org = persp.imagepoints(img,r,n_pts,t,
                                                                col_min,col_max)
        else:
            break
#--------------------------- 3.b Get Object Points      -----------------------#
        #-- get points --#
        pts_obj, margin = persp.objectpoints(n_pts)
        img_test,pts_obj,size = persp.format_points(pts_obj,margin)
        img_flat,M = persp.geometric_transformationN(img,pts_obj,pts_img,size)

        #-- save results --#
        current_time = str(int(time.mktime(time.gmtime())))
        name='ref_' +str(n)+'_'+current_time+str('.txt')
        persp.write_ref(out_dir,name,pts_img,M,pts_obj)
#--------------------------- 3.c Calibrate              -----------------------#
        img = calib.Image(n)
        img.read_ref(out_dir,"ref_",n_pts)
        # add z component to ref_real
        img.ref_real = img.augment(img.ref_real,ref_z)
        cam.reposition(img.ref_real,img.ref_img,0,flag)
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
            t = 100
            img_red,circ_red,p,th_red = persp.imagepoints(img,r,1,t,
                                                          col_min,col_max)
            #-- save resutls --#
            name='pos_img'+str(n)+'.txt'
            persp.write_pos(out_dir,name,p)

#--------------------------- 4.b Calculate Object Point -----------------------#
        cams = dict()
        pts = dict()
        for i,n in enumerate(n_cameras):
            cam = calib.Camera(n)
            cam.read(cam_dir)
            img = calib.Image(n)
            img.read_ref(out_dir,"ref_",n_pts)
            img.read_pos(out_dir,"pos_img")
            img.ref_real = img.augment(img.ref_real,ref_z)
            cam.reposition(img.ref_real,img.ref_img,0,flag)
            img.r,err2,err3 = calib.get_leastsquares([cam],[img.augment(img.r_img)],
                                                     'my',r_height,p_real)
            img.ref,err_img = cam.check_points(img.ref_real,img.ref_img)
            #print("Ref point match [px]",cam.n,"\n",err_img)
            #TODO: Check if ref match is good enough for this camera!
            cams[i] = cam
            pts[i] = img.augment(img.r_img)

        # For all permutations (when more than 1 camera)
        p_lq,err2,err3 = calib.get_leastsquares(cams.values(),pts.values(),
                                                'hz',r_height,p_real)
        print("LQ fixed height [mm]: ",p_lq.T,"error 2D: ",err2,"/ 3D: ",err3)
        p_lq2,err2,err3 = calib.get_leastsquares(cams.values(),pts.values(),
                                                 'hz','',p_real)
        print("LQ free height [mm]:  ",p_lq2.T,"error 2D: ",err2,"/ 3D: ",err3)
        print("Real position:        ",p_real)
        choice = raw_input("Do you want to locate the robot? (y/n) ")

#--------------------------- 4.c Make Robot Move        -----------------------#
