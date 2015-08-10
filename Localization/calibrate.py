#!/usr/bin/env python
from __future__ import division
import numpy as np
import cv2
import matplotlib.pyplot as plt
# built-in modules
import os

USAGE = '''
USAGE: calib.py [--save <output path>] [--square_size <square size>] [--n <camera_number>]]
'''

if __name__ == '__main__':
    import sys
    import getopt
    from glob import glob
    import get_image

    save_dir = ''
    square_size = 1
    n = 141
    opts,args = getopt.getopt(sys.argv[1:], '', ['save=', 'square_size=','n='])
    for opt, arg in opts:
        if opt == '--save':
            save_dir = arg
        elif opt == '--square_size':
            square_size = float(arg)
        elif opt == '--n':
            n = int(arg)

    pattern_size = (9, 6)
    pattern_points = np.zeros( (np.prod(pattern_size), 3), np.float32 )
    pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points *= square_size

    obj_points = []
    img_points = []
    h, w = 0, 0
    counter = 1
    choice = "y"
    while True:
        try:
            if choice == "y":
                img = get_image.get_image(n)
                print("image loaded okay")
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                sys.exit(1)
                h, w = gray.shape[:2]
                found, corners = cv2.findChessboardCorners(gray, pattern_size)
                if found:
                    print("chessboard found")
                    term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1 )
                    cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), term)
                    img_points.append(corners.reshape(-1, 2))
                    obj_points.append(pattern_points)
                    if save_dir:
                        print("saving...")
                        vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                        cv2.drawChessboardCorners(vis, pattern_size, corners, found)
                        cv2.imwrite('%s/output%d_%d.jpg'.format(save_dir,n,counter),img)
                        cv2.imwrite('%s/input%d_%d.jpg'.format(save_dir,n,counter), vis)
                elif not found:
                    print('chessboard not found')
                counter += 1
                choice = raw_input("Do you want to perform another calibration? (y/n)")
            elif choice == "n":
                rms, cam, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)
                print("RMS:", rms)
                print("camera matrix:\n", cam)
                print("distortion coefficients:",dist)
                sys.exit(1)
        except KeyboardInterrupt:
            print("program terminated by user")
            sys.exit(1)

