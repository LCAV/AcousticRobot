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
USAGE: locate.py -o <output path> -n <number of points> [-f <fisheye on (0/1)>]
'''
ROBOT_LOC='''Robot Localization   :
Enter choice ('n' or 'o' to use new or old measurement, 'q' to quit): '''
EXTRINSIC='''Extrinsic Calibration:
Enter camera number or 'q' to skip: '''
INTRINSIC='''Intrinsic Calibration:
Enter camera number or 'q' to skip: '''
def get_param():
    ''' returns parameters from command line '''
    global USAGE
    out_dir = ''
    m = 0
    fisheye = 0
    try:
        opts,args = getopt.getopt(sys.argv[1:],"o:m:f:",
                                  ['output=','m=','fisheye='])
    except getopt.GetoptError:
        print(USAGE)
        sys.exit(2)
    for opt, arg in opts:
        if opt in ('--output',"-o"):
            out_dir = str(arg)
        if opt in ('--m',"-m"):
                m = int(arg)
        if opt in ('--fisheye',"-f"):
                fisheye = int(arg)
    if out_dir[-1]!="/":
        out_dir = out_dir+"/"
    return out_dir,m,fisheye

if __name__ == '__main__':
    import sys
    import getopt
#---------------------------       Initialization       -----------------------#
    cam_dir = 'calib/'
    out_dir,n_pts,fisheye = get_param() # number of reference points
    n_cameras = [141]
    #n_cameras = [139,141,145]
    #p_real = np.matrix([750,790,190])
    p_real = np.matrix([645,930,180])
    C139_real = np.matrix([110,670,1750])
    C141_real = np.matrix([460,40,1320])
    r_height = p_real[0,2] #height of robot in mm

    ref_z = 20*np.ones((1,n_pts))
    #ref_z = np.array([135,0,230,0]) #height of reference points in mm
    #ref_z = '' #height automatically set to 0
    flag = 0 # alorithm for solvepnp
    ransac = 0 # use ransac or not
    repErr_range = range(1,50)# for Ransac algorithm (8 by default)
#--------------------------- 0. Intrinsic Calibration   -----------------------#

    n = raw_input(INTRINSIC)
    if n!='q':
        n=int(n)
        cam = calib.Camera(n)
        img_points,obj_points,size = cam.get_checkpoints(out_dir,fisheye)
        cam.calibrate(obj_points,img_points,size)
        cam.save(cam_dir,fisheye)
#--------------------------- 3. Extrinsic Calibration   -----------------------#
#--------------------------- 3.a Get Image Points       -----------------------#
    n = ''
    while True:
        n = raw_input(EXTRINSIC)
        if n != 'q':
            plt.close('all')
            n = int(n)
            cam = calib.Camera(n)
            cam.read(cam_dir,fisheye)
            img = get.get_image(n)

            #-- undistort image --#
            img = cam.undistort(img)

            #-- get points --#
            col_min = np.array([175,100,0],dtype=np.uint8)
            col_max = np.array([20,250,255],dtype=np.uint8)
            r = 30
            t = 50
            img_org,circ_org,pts_img,th_org = persp.imagepoints(img,r,n_pts,t,
                                                                col_min,col_max)
        else:
            break
#--------------------------- 3.b Get Object Points      -----------------------#
        #-- get points --#
        pts_obj, margin,M = persp.objectpoints(6)
        img_test,pts_obj,size = persp.format_points(pts_obj,margin)
        img_flat,M = persp.geometric_transformationN(img,pts_obj,pts_img,size)

        #-- save results --#
        # positions
        current_time = str(int(time.mktime(time.gmtime())))
        name='ref_' +str(n)+'_'+current_time+str('.txt')
        persp.write_ref(out_dir,name,pts_img,M,pts_obj)
        # images
        imgs = {'img':img}
        persp.visualization(imgs)
        imgs = {'img_org':img_org,'circ_org':circ_org}
        persp.visualization(imgs)
        img_summary = persp.create_summary(img_flat,pts_obj)
        imgs = {'summary':img_summary}
        persp.visualization(imgs,0,1)
        persp.save_open_images(out_dir,n)

#--------------------------- 4. Localization            -----------------------#
#--------------------------- 4.a Get Image Points       -----------------------#
    choice = raw_input(ROBOT_LOC)
    while choice != "q":
        plt.close('all')
        # save new position in file
        if choice == 'n':
            for n in n_cameras:
                cam = calib.Camera(n)
                cam.read(cam_dir,fisheye)

                img = get.get_image(n)

                #-- undistort image --#
                img = cam.undistort(img)
                #-- get point --#
                #-- get last reference points --#
                img_ref = calib.Image(n)
                img_ref.read_ref(out_dir,"ref_",n_pts)
                col_min = np.array([150,100,0],dtype=np.uint8)
                col_max = np.array([20,250,255],dtype=np.uint8)
                r_rob = 40
                r_ref = 30
                t = 50
                img_red,circ_red,p,th_red = persp.imagepoints_auto(img,r_rob,1,t,
                                                              col_min,col_max,
                                                              img_ref.ref_img,r_ref)
                #-- save resutls --#
                name='pos_img'+str(n)+'.txt'
                persp.write_pos(out_dir,name,p)

#--------------------------- 4.b Calculate Object Point -----------------------#
        cams = dict()
        imgs = dict()
        pts = dict()
        errs = dict()
        for i,n in enumerate(n_cameras):
            # Load camera
            cam = calib.Camera(n)
            cam.read(cam_dir,fisheye)
            img = calib.Image(n)
            img.read_ref(out_dir,"ref_",n_pts)
            img.read_pos(out_dir,"pos_img")
            img.ref_real = img.augment(img.ref_real,ref_z)

            #--- Extrinsic calibration ---#
            err_img = 0
            if ransac:
                err_img = cam.ransac_loop(img,flag,repErr_range)
            else:
                cam.reposition(img.ref_real,img.ref_img,0,flag,ransac)
                ref, err_img = cam.check_imagepoints(img.augment(img.ref_real),
                                                     img.ref_img)

            #--- Individual Robot position ---#
            img.r,err2,err3 = calib.get_leastsquares([cam],[img.augment(img.r_img)],
                                                    'my',r_height,p_real)
            imgs[i]=img
            cams[i]=cam
            pts[i]=img.augment(img.r_img)

        # For all permutations (when more than 1 camera)
        p_lq,err2,err3 = calib.get_leastsquares(cams.values(),pts.values(),
                                                'hz',r_height,p_real)
        p_lq2,err22,err32 = calib.get_leastsquares(cams.values(),pts.values(),
                                                 'hz','',p_real)


        # Results visualization and saving
        msg1="Fixed height [mm]: [{0:8.4f} , {1:8.4f} , {2:8.4f}], error 2D: {3:5.2f} 3D: {4:5.2f}".format(float(p_lq[0]),float(p_lq[1]),float(p_lq[2]),err2,err3)
        msg2="Free height [mm]:  [{0:8.4f} , {1:8.4f} , {2:8.4f}], error 2D: {3:5.2f} 3D: {4:5.2f}".format(float(p_lq2[0]),float(p_lq2[1]),float(p_lq2[2]),err22,err32)
        msg3="Real position [mm]:[{0:8.4f} , {1:8.4f} , {2:8.4f}]".format(p_real[0,0],p_real[0,1],p_real[0,2])

        print(msg1)
        print(msg2)
        print(msg3)
        with open(out_dir+"results.txt",'w') as f:
            f.write(msg1+"\n")
            f.write(msg2+"\n")
            f.write(msg3+"\n")

        choice = raw_input(ROBOT_LOC)

#--------------------------- 4.c Make Robot Move        -----------------------#
