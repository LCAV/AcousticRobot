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
import cv2.cv as cv
import cv2
import time
import move
import Audio
USAGE = '''
USAGE: locate.py -o <output path> -i <input path> [-m <number of points>]
[-f <fisheye on (0/1)>] [-d for debugging] [-h to show this help]

default:
m=6
f='0000'
d=0

In the debug mode, instead of accessing the cameras, a picture is loaded from the input path folder.
Name the picture has to be named N_imageX.png, where N is the step number (starting from 0)
and X is the camera number. for the extrinsic calibration, the image '_imageX.png' is used.
'''
DEBUG=0

EXTRINSIC='''Extrinsic calibration:
Enter camera number or 'q' to skip: '''
INTRINSIC='''Intrinsic calibration:
Enter camera number or 'q' to skip: '''
ROBOT_LOC='''Perform localization? ('y' or 'old' to use new or old measurement, 'n' to quit):
'''
ROBOT_VIS='''Visual localization :
Localize robot using cameras?  ('y' for yes or 'n' for no)
'''
ROBOT_ODO='''Odometry localization :
Localize robot using odometry?  ('y' for yes or 'n' for no)
'''
ROBOT_ACO='''Acoustic localization :
Localize robot using acoustics? ('y' for yes or 'n' for no)
'''
ROBOT_MOVE='''Moving Robot         :
Do you want to move the robot? ('y' for yes or 'n' for no)
'''
ROBOT_REAL='''Real Robot           :
Do you want to save the real position? ('y' for yes or 'n' for no)
'''
# color frame ƒor robot detection
MIN = np.array([0,100,0],dtype=np.uint8)
MAX = np.array([10,250,255],dtype=np.uint8)
# color frame for reference poitn detection
MIN_REF = np.array([0,150,0],dtype=np.uint8)
MAX_REF = np.array([10,250,255],dtype=np.uint8)
R_REF = 8 # radius of referencepoints (in pixels)
R_ROB = 15 # radius of robot point (in pixels)
THRESH = 50 # empiric threshold for circle detection
MARGIN = np.array([10,10],dtype=np.float) #margin from leftmost and downlost ref point to reference (in mm)
PTS_BASIS = np.array(([145,270],[1240,150])) #position of first and second reference points from wall (in mm)
R_HEIGHT = 20
NCAMERAS = [139,141,143,145]
CHECKERBOARD = 1 # weather or not to use checkerboard for extrinsic calculation
WIDTH = 7
HEIGHT = 5

# Audio
N_CHANNELS=2
RATE = 44100
CHUNK = 1024
def get_param():
    global DEBUG
    ''' returns parameters from command line '''
    out_dir = ''
    in_dir = ''
    m = 6
    fisheye = '00001'
    try:
        opts,args = getopt.getopt(sys.argv[1:],"o:i:m:f:d",
                                  ['output=','input=','m=','fisheye=','debug='])
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
        if opt in ('--debug',"-d"):
            DEBUG=1
        if opt in ('--fisheye',"-f"):
            # array with '' for 0 and '1' for 1
            fish = np.array(arg.split('0'))
            # array with True and False
            fisheye = [c=='1' for c in fish]
            print("Fisheye settings for [139,141,143,145]:",fisheye[:4])
    if opts==[]:
        print(USAGE)
        sys.exit(2)
    if out_dir[-1]!="/":
        out_dir = out_dir+"/"
    if in_dir[-1]!="/":
        in_dir = in_dir+"/"
    return out_dir,in_dir,m,fisheye
def signal_handler(signal, frame):
    ''' Interrupt handler for stopping on KeyInterrupt '''
    print('Program stopped manually')
    sys.exit(2)

if __name__ == '__main__':
    robot_connected=False
   # try:
    import sys
    import getopt
#---------------------------       Initialization       -------------------#
    # General
    out_dir,in_dir,NPTS,fisheye = get_param()
    if DEBUG:
        print("Running in DEBUG mode\n")
    else:
        print("Running in real mode\n")

    if CHECKERBOARD:
        NPTS=WIDTH*HEIGHT

    TIME = str(int(time.mktime(time.gmtime())))
    input_au =in_dir+"sound.wav"
    input_mov = in_dir+"control.txt"
    input_obj = in_dir+"objectpoints.csv"
    output_odo = out_dir+"odometry_"+TIME+".txt"
    output_tim = out_dir+"timings_"+TIME+".txt"
    output_au = out_dir+"audio_"+TIME+".wav"
    # Visual localization
    flag = 0 # alorithm for solvepnp
    numbers=range(2,4) # number of cameras to loop through
    r_wall = np.matrix([0,0,R_HEIGHT]) # real robot position from wall
    r_real = persp.change_wall_to_ref(PTS_BASIS,MARGIN,r_wall.copy())

    R_HEIGHT = r_real[0,2] #height of robot in mm
    choice_ref = 4 #chosen reference point for error calculation
    ref_z = 20*np.ones((1,NPTS))
    #ref_z = np.array([135,0,230,0]) #height of reference points in mm
    #ref_z = '' #height automatically set to 0

    # Odometry
    loop_counter = ''

#--------------------------- 0. Intrinsic Calibration   -------------------#

    n = raw_input(INTRINSIC)
    if n!='q':
        n=int(n)
        i=NCAMERAS.index(n)
        cam = calib.Camera(n)
        img_points,obj_points,size = cam.get_checkpoints(out_dir,5,8,fisheye)
        cam.calibrate(obj_points,img_points,size)
        cam.save(in_dir,fisheye[i])
#--------------------------- 3. Extrinsic Calibration   -------------------#
    n = ''
    while True:
        n = raw_input(EXTRINSIC)
        if n != 'q':
            plt.close('all')
            n = int(n)
            cam = calib.Camera(n)
            cam.read(in_dir,fisheye)
            img=calib.Image(n)
            if DEBUG:
                img.load_image(in_dir+str(loop_counter)+"_image"+str(n)+".png")
            else:
                img.take_image()

            # save unchanged image
            plt.imsave(out_dir+str(loop_counter)+'_image_'+str(n)+'_'+TIME,img.img)
            if not CHECKERBOARD:
                img.get_refimage(R_REF,THRESH,MIN_REF,MAX_REF,NPTS,1,out_dir,
                                TIME)
                img.get_refobject(input_obj,NPTS,MARGIN,1,out_dir,TIME)
            else:
                img.get_checkerboard(in_dir,fisheye,WIDTH,HEIGHT,MARGIN,
                                        R_REF,THRESH,MIN_REF,
                                        MAX_REF,1,out_dir,TIME)
            name='ref_'+str(n)+'_'+TIME+str('.txt')
            img.write_ref(out_dir,name,img.ref_img,img.M,img.ref_obj)
        else:
            break

#--------------------------- 4. Localization        -----------------------#
    loop_counter = 0
    times=0
    commands=''
    choice_loc = raw_input(ROBOT_LOC)
    while choice_loc != "n":
#--------------------------- 4.1 Real Position    -------------------------#
        choice = raw_input(ROBOT_REAL)
        if choice != 'n':
            x = raw_input("robot position x in mm, measured from wall: ")
            y = raw_input("robot position y in mm, measured from wall: ")
            if x!='':
                r_wall[0,0]=x
            if y!='':
                r_wall[0,1]=y
            r_real = persp.change_wall_to_ref(PTS_BASIS,MARGIN,r_wall.copy())
            name='posreal_'+TIME+'.txt'
            persp.write_pos(out_dir,name,np.array(r_wall)[0])
            name='posreal_ref_'+TIME+'.txt'
            persp.write_pos(out_dir,name,np.array(r_real)[0])

#--------------------------- 4.1 Visual localization ----------------------#
#--------------------------- 4.1.a Get Image Points  ----------------------#

        choice = raw_input(ROBOT_VIS)
        if choice != 'n':
            print("Visual localization...")
            plt.close('all')
            # save new robot position in file
            if choice == 'y':
                for i,n in enumerate(NCAMERAS):
                    img = calib.Image(n)

                    if DEBUG:
                        img.load_image(in_dir+str(loop_counter)+"_image_"+str(n)+".png")
                    else:
                        img.take_image()
                    # save unchanged image
                    plt.imsave(out_dir+str(loop_counter)+'_image_'+str(n)+'_'+TIME,img.img)

                    img.get_robotimage(R_ROB,THRESH,MIN,MAX,1,out_dir,TIME,loop_counter)
                    name='posimg_'+str(n)+'_'+TIME+'.txt'
                    img.write_pos(out_dir,name,img.r_img)
#--------------------------- 4.1.b Calculate Object Point -----------------#
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
                img.read_ref(out_dir,"ref_",NPTS)
                img.read_pos(out_dir,"posimg_")
                img.ref_obj = img.augment(img.ref_obj,ref_z)

                #--- Extrinsic calibration ---#
                err_img = 0
                cam.reposition(img.ref_obj,img.ref_img,flag)
                ref, err_img,lamda = cam.check_imagepoints(img.augment(img.ref_obj),
                                                        img.ref_img)
                ref_obj,err_obj = cam.check_objectpoints(img.augment(img.ref_obj),
                                                        img.augment(img.ref_img))
                #--- Individual Robot position ---#
                img.r_obj,err2,err3 = calib.get_leastsquares([cam],[img.augment(img.r_img)],
                                                        'hz',R_HEIGHT,r_real)
                imgs[i]=img
                cams[i]=cam
                pts[i]=img.augment(img.r_img)
                pts_ref[i]=img.augment(img.ref_img)[choice_ref,:]
                errs[i]=err_img

            errors = np.matrix([round(np.sum(x)/len(x),4) for x in errs.values()])

            ref_obj = img.ref_obj[choice_ref,:] # real position of reference points
            p1,e21,e31,arr1 = calib.get_leastsquares_combinations(NCAMERAS,numbers,cams.values(),
                                            pts_ref.values(),'hz',ref_obj[0,2],ref_obj)
            calib.save_combinations(out_dir,"combi_ref_fixed"+str(choice_ref),arr1,p1,(e21,e31))
            p2,e22,e32,arr2 = calib.get_leastsquares_combinations(NCAMERAS,numbers,cams.values(),
                                            pts_ref.values(),'hz','',ref_obj)
            calib.save_combinations(out_dir,"combi_ref_free"+str(choice_ref),arr2,p2,(e22,e32))
            p3,e23,e33,arr3 = calib.get_leastsquares_combinations(NCAMERAS,numbers,cams.values(),
                                            pts.values(),'hz',R_HEIGHT,r_real)
            calib.save_combinations(out_dir,"combi_rob_fixed"+str(choice_ref),arr3,p3,(e23,e33))
            p4,e24,e34,arr4 = calib.get_leastsquares_combinations(NCAMERAS,numbers,cams.values(),
                                            pts.values(),'hz','',r_real)
            calib.save_combinations(out_dir,"combi_rob_free"+str(choice_ref),arr4,p4,(e24,e34))
            calib.save_combinations(out_dir,"combi_all"+str(choice_ref),arr4,p2,(e21,e22,e32,
                                                                e23,e24,e34),fisheye)
            # Pick best combination based on chosen criteria
            index2,err2 = calib.get_best_combination(e31)
            p_lq1 = p3[index2]
            err21 = e23[index2]
            err31 = e33[index2]
            p_lq2 = p4[index2]
            err22 = e24[index2]
            err32 = e34[index2]
            print("real position:",r_real)

            # Results visualization and saving
            msg0="Best combination (based on best 3D error with fixed height):{0},error: {1:6.4f}".format(arr2[index2],err2)
            msg1="Fixed height [mm]: [{0:8.4f} , {1:8.4f} , {2:8.4f}], error 2D: {3:5.2f} 3D: {4:5.2f}".format(float(p_lq1[0]),float(p_lq1[1]),float(p_lq1[2]),err21,err31)
            msg2="Free height [mm]:  [{0:8.4f} , {1:8.4f} , {2:8.4f}], error 2D: {3:5.2f} 3D: {4:5.2f}".format(float(p_lq2[0]),float(p_lq2[1]),float(p_lq2[2]),err22,err32)
            msg3="Real position [mm]:[{0:8.4f} , {1:8.4f} , {2:8.4f}]".format(r_real[0,0],r_real[0,1],r_real[0,2])
            msg4="Error reference points [px]: {0} ".format(errors)
            print(msg0)
            print(msg1)
            print(msg2)
            print(msg3)
            print(msg4)

            name='posobj_fix_'+str(choice_ref)+'_'+TIME+'.txt'
            p_lq1_wall = persp.change_ref_to_wall(PTS_BASIS,MARGIN,p_lq1.T[0])
            persp.write_pos(out_dir,name,p_lq1_wall)
            name='posobj_free_'+str(choice_ref)+'_'+TIME+'.txt'
            p_lq2_wall = persp.change_ref_to_wall(PTS_BASIS,MARGIN,p_lq2.T)
            persp.write_pos(out_dir,name,p_lq2_wall)

            with open(out_dir+str(loop_counter)+"_results_"+
                      str(choice_ref)+'_'+TIME+".txt",'w') as f:
                f.write(msg0+"\n")
                f.write(msg1+"\n")
                f.write(msg2+"\n")
                f.write(msg3+"\n")
                f.write(msg4+"\n")

#--------------------------- 4.2 Odometry Localization   ------------------#
        choice = raw_input(ROBOT_ODO)
        if choice == 'y':

            # Connect to robot
            if not robot_connected:
                Robot = move.Robot()
                Robot.connect()
                robot_connected = True

            print("Odometry localization")
            Robot.get_position(output_odo)
#--------------------------- 4.3 Acoustic Localization   ------------------#
        choice = raw_input(ROBOT_ACO)
        if choice == 'y':
            print("Acoustic localization")
            out_au = output_au.replace('/','/'+str(loop_counter)+'_')
            Au = Audio.Audio(input_au,out_au,N_CHANNELS,3,RATE,CHUNK)
            frames=Au.play_and_record()
            Au.save_wav_files(frames)
#--------------------------- 5. Make Robot Move        --------------------#
        choice = raw_input(ROBOT_MOVE)
        if choice == 'y':
            if not robot_connected:
                Robot = move.Robot()
                Robot.connect()
                robot_connected = True
            if commands=='':
                (times, commands) = move.read_file(Robot,input_mov)
            if loop_counter > max(times.keys()):
                print("End of control input file reached.")
                break

            print("Activating motors")
            Robot.activate()
            time.sleep(2)
            print("Moving robot")
            t = times[loop_counter]
            c = commands[loop_counter]
            Robot.move(t,c,output_tim)

        loop_counter += 1
        choice_loc = raw_input(ROBOT_LOC)
#--------------------------- 6. Terminate               -------------------#
    # disconnect robot in the end
    if robot_connected:
        Robot.cleanup()
        print("Robot cleaned up successfully")
    print("Program terminated")
    '''except: # disconnect robot when an error occurs
        if robot_connected:
            Robot.cleanup()
            print("Robot cleaned up successfully")
        print("Program terminated")
    '''
