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
import move
import Audio
USAGE = '''
USAGE: locate.py -o <output path> -i <input path> [-m <number of points>] [-f <fisheye on (0/1)>]
default:
m=6
f=0

'''
EXTRINSIC='''Extrinsic calibration:
Enter camera number or 'q' to skip: '''
INTRINSIC='''Intrinsic calibration:
Enter camera number or 'q' to skip: '''
ROBOT_LOC='''Localization          :
Perform localization? ('y' or 'old' to use new or old measurement, 'n' to quit): '''
ROBOT_VIS='''Visual localization :
Localize robot using cameras?  ('y' for yes or 'n' for no)'''
ROBOT_ODO='''Odometry localization :
Localize robot using odometry?  ('y' for yes or 'n' for no)'''
ROBOT_ACO='''Acoustic localization :
Localize robot using acoustics? ('y' for yes or 'n' for no)'''
ROBOT_MOVE='''Moving Robot         :
Do you want to move the robot? ('y' for yes or 'n' for no)'''
# color frame ƒor robot detection
COL_MIN = np.array([150,100,0],dtype=np.uint8)
COL_MAX = np.array([20,250,255],dtype=np.uint8)
# color frame for reference poitn detection
COL_MIN_REF = np.array([175,100,0],dtype=np.uint8)
COL_MAX_REF = np.array([20,250,255],dtype=np.uint8)

NCAMERAS = [139,141,143,145]
def get_param():
    ''' returns parameters from command line '''
    out_dir = ''
    in_dir = ''
    m = 6
    fisheye = '0000'
    try:
        opts,args = getopt.getopt(sys.argv[1:],"o:i:m:f:",
                                  ['output=','input=','m=','fisheye='])
    except getopt.GetoptError:
        print(USAGE)
        sys.exit(2)
    for opt, arg in opts:
        if opt == 'h':
            print(USAGE)
            sys.exit(2)
        if opt in ('--output',"-o"):
            out_dir = str(arg)
        if opt in ('--input',"-i"):
            in_dir = str(arg)
        if opt in ('--m',"-m"):
            m = int(arg)
        if opt in ('--fisheye',"-f"):
            # array with '' for 0 and '1' for 1
            fish = np.array(arg.split('0'))
            # array with True and False
            fisheye = [c=='1' for c in fish]
    if opts==[]:
        print(USAGE)
        sys.exit(2)
    if out_dir[-1]!="/":
        out_dir = out_dir+"/"
    if in_dir[-1]!="/":
        in_dir = in_dir+"/"
    return out_dir,in_dir,m,fisheye
def signal_handler(signal, frame):
    ''' Interrupt handler for stopping on KayInterrupt '''
    print('Program stopped manually')
    sys.exit(2)

if __name__ == '__main__':
    import sys
    import getopt
#---------------------------       Initialization       -----------------------#
    # General
    out_dir,in_dir,n_pts,fisheye = get_param()
    start_time = str(int(time.mktime(time.gmtime())))
    input_au =in_dir+"sound.wav"
    input_mov = in_dir+"control.tex"
    input_obj = in_dir+"objectpoints.csv"
    output_odo = out_dir+"odometry_"+start_time+".tex"
    output_tim = out_dir+"timings_"+start_time+".tex"
    output_au = out_dir+"audio_"+start_time+".wav"
    # Visual localization
    flag = 0 # alorithm for solvepnp
    numbers=range(2,4) # number of cameras to loop through

    r_ref = 10 # radius of referencepoints (in pixels)
    r_rob = 10 # radius of robot point (in pixels)
    t = 50 # empiric threshold for circle detection
    r_real = np.matrix([1822,1516,182]) #real position robot in mm
    r_height = r_real[0,2] #height of robot in mm
    choice_ref = 4 #chosen reference point for error calculation
    ref_z = 20*np.ones((1,n_pts))
    #ref_z = np.array([135,0,230,0]) #height of reference points in mm
    #ref_z = '' #height automatically set to 0

    # Odometry localization
    Robot = move.Robot()
    (times, commands) = move.read_file(Robot,input_mov)
    Robot.connect()
    loop_counter = 0
#--------------------------- 0. Intrinsic Calibration   -----------------------#

    n = raw_input(INTRINSIC)
    if n!='q':
        n=int(n)
        i=NCAMERAS.index(n)
        cam = calib.Camera(n)
        img_points,obj_points,size = cam.get_checkpoints(out_dir,fisheye[i])
        cam.calibrate(obj_points,img_points,size)
        cam.save(in_dir,fisheye[i])
#--------------------------- 3. Extrinsic Calibration   -----------------------#
#--------------------------- 3.a Get Image Points       -----------------------#
    n = ''
    while True:
        n = raw_input(EXTRINSIC)
        if n != 'q':
            plt.close('all')
            n = int(n)
            cam = calib.Camera(n)
            cam.read(in_dir,fisheye)
            img = get.get_image(n)

            #-- undistort image --#
            img = cam.undistort(img)

            #-- get points --#
            img_org,circ_org,pts_img,th_org = persp.imagepoints(img,r_ref,n_pts,t,
                                                                COL_MIN_REF,COL_MAX_REF)
        else:
            break
#--------------------------- 3.b Get Object Points      -----------------------#
        #-- get points --#
        pts_obj, margin,M = persp.objectpoints(n_pts,input_obj)
        img_test,pts_obj,size = persp.format_points(pts_obj,margin)
        img_flat,M = persp.geometric_transformationN(img,pts_obj,pts_img,size)

        #-- save results --#
        # positions
        name='ref_'+str(n)+'_'+start_time+str('.txt')
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
#--------------------------- 4.1 Visual localization    -----------------------#
#--------------------------- 4.1.a Get Image Points     -----------------------#
    choice_loc = raw_input(ROBOT_LOC)
    while choice_loc != "n":
        choice = raw_input(ROBOT_VIS)
        if choice != 'n':
            print("Visual localization")
            plt.close('all')
            # save new reference positions in file
            if choice == 'y':
                for i,n in enumerate(NCAMERAS):
                    cam = calib.Camera(n)
                    cam.read(in_dir,fisheye[i])

                    img = get.get_image(n)

                    #-- undistort image --#
                    img = cam.undistort(img)
                    #-- get point --#
                    #-- get last reference points --#
                    img_ref = calib.Image(n)
                    img_ref.read_ref(out_dir,"ref_",n_pts)
                    #img_red,circ_red,p,th_red = persp.imagepoints_auto(img,r_rob,1,t,
                    #                                              COL_MIN,COL_MAX,
                    #                                              img_ref.ref_img,r_ref)
                    img_red,circ_red,p,th_red = persp.imagepoints(img,r_rob,1,t,
                                                                COL_MIN,COL_MAX)
                    #--save results--#
                    name='posimg_'+str(n)+'_'+start_time+'.txt'
                    persp.write_pos(out_dir,name,p)
                    img={'img_red':img_red,'circ_red':circ_red,'img':img}
                    persp.visualization(imgs)
                    persp.save_open_images(out_dir,n,loop_counter)

    #--------------------------- 4.1.b Calculate Object Point ---------------------#
            cams = dict()
            imgs = dict()
            pts = dict()
            pts_ref = dict()
            errs = dict()
            for i,n in enumerate(NCAMERAS):
                # Load camera
                cam = calib.Camera(n)
                cam.read(in_dir,fisheye[i])
                img = calib.Image(n)
                img.read_ref(out_dir,"ref_",n_pts)
                img.read_pos(out_dir,"posimg")
                img.ref_real = img.augment(img.ref_real,ref_z)

                #--- Extrinsic calibration ---#
                err_img = 0
                cam.reposition(img.ref_real,img.ref_img,flag)
                ref, err_img,lamda = cam.check_imagepoints(img.augment(img.ref_real),
                                                           img.ref_img)
                ref_obj, err_obj = cam.check_objectpoints(img.augment(img.ref_real),
                                                          img.augment(img.ref_img))
                #--- Individual Robot position ---#
                img.r,err2,err3 = calib.get_leastsquares([cam],[img.augment(img.r_img)],
                                                        'hz',r_height,r_real)
                imgs[i]=img
                cams[i]=cam
                pts[i]=img.augment(img.r_img)
                pts_ref[i]=img.augment(img.ref_img)[choice_ref,:]
                errs[i]=err_img

            ref_real = img.ref_real[choice_ref,:] # real position of reference points
            p1,e21,e31,arr1 = calib.get_leastsquares_combinations(NCAMERAS,numbers,cams.values(),
                                            pts_ref.values(),'hz',ref_real[0,2],ref_real)
            calib.save_combinations(out_dir,"combi_ref_fixed"+str(choice_ref),arr1,p1,(e21,e31),fisheye)
            p2,e22,e32,arr2 = calib.get_leastsquares_combinations(NCAMERAS,numbers,cams.values(),
                                            pts_ref.values(),'hz','',ref_real)
            calib.save_combinations(out_dir,"combi_ref_free"+str(choice_ref),arr2,p2,(e22,e32),fisheye)
            p3,e23,e33,arr3 = calib.get_leastsquares_combinations(NCAMERAS,numbers,cams.values(),
                                            pts.values(),'hz',r_height,r_real)
            calib.save_combinations(out_dir,"combi_rob_fixed"+str(choice_ref),arr3,p3,(e23,e33),fisheye)
            p4,e24,e34,arr4 = calib.get_leastsquares_combinations(NCAMERAS,numbers,cams.values(),
                                            pts.values(),'hz','',r_real)
            calib.save_combinations(out_dir,"combi_rob_free"+str(choice_ref),arr4,p4,(e24,e34),fisheye)
            calib.save_combinations(out_dir,"combi_all"+str(choice_ref),arr4,p2,(e21,e22,e32,
                                                                e23,e24,e34),fisheye)
            # Best combination based on chosen criteria
            index2,err2 = calib.get_best_combination(e32)

            # For all permutations (when more than 1 camera)
            p_lq1 = p3[index2]
            err21 = e23[index2]
            err31 = e33[index2]
            p_lq2 = p4[index2]
            err22 = e24[index2]
            err32 = e34[index2]
            errors = np.matrix([round(np.sum(x)/len(x),4) for x in errs.values()])
            # Results visualization and saving
            msg0="Best combination (based on best 3D error with free height):{0},error: {1:6.4f}".format(arr2[index2],err2)
            msg1="Fixed height [mm]: [{0:8.4f} , {1:8.4f} , {2:8.4f}], error 2D: {3:5.2f} 3D: {4:5.2f}".format(float(p_lq1[0]),float(p_lq1[1]),float(p_lq1[2]),err21,err31)
            msg2="Free height [mm]:  [{0:8.4f} , {1:8.4f} , {2:8.4f}], error 2D: {3:5.2f} 3D: {4:5.2f}".format(float(p_lq2[0]),float(p_lq2[1]),float(p_lq2[2]),err22,err32)
            msg3="Real position [mm]:[{0:8.4f} , {1:8.4f} , {2:8.4f}]".format(r_real[0,0],r_real[0,1],r_real[0,2])
            msg4="Error reference poitns [px]: {0} ".format(errors)
            print(msg0)
            print(msg1)
            print(msg2)
            print(msg3)
            print(msg4)
            name='posobj_'+start_time+'.txt'
            persp.write_pos(out_dir,name,p)


            #with open(out_dir+"results"+str(choice_ref)+".txt",'w') as f:
            #    f.write(msg0+"\n")
            #    f.write(msg1+"\n")
            #    f.write(msg2+"\n")
            #    f.write(msg3+"\n")
            #    f.write(msg4+"\n")

#--------------------------- 4.2 Odometry Localization   ----------------------#
        choice = raw_input(ROBOT_ODO)
        if choice == 'y':
            print("Odometry localization")
            Robot.get_position(output_odo)
#--------------------------- 4.3 Acoustic Localization   ----------------------#
        choice = raw_input(ROBOT_ACO)
        if choice == 'y':
            print("Acoustic localization")
            output_au = output_au.replace('/','/'+str(loop_counter)+'_')
            Au = Audio.Audio(input_au,output_au,1)
            frames=Au.play_and_record()
            Au.save_wav_files(frames)
#--------------------------- 5. Make Robot Move        -----------------------#
        choice = raw_input(ROBOT_MOVE)
        if choice == 'y':
            print("Moving robot")
            t = times[loop_counter]
            c = commands[loop_counter]
            Robot.move(t,c,output_tim)

        loop_counter += 1
        choice_loc = raw_input(ROBOT_LOC)
#--------------------------- 6. Terminate               -----------------------#
    Robot.cleanup()
    print("Robot cleaned up successfully")
