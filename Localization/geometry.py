# -*- coding: utf-8 -*-
from __future__ import division, print_function
import numpy as np
import matplotlib.pyplot as plt
import csv
import getopt
import sys

def get_parameters():
    fname = ''
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
    with open(fname,"r") as f:
        c = csv.reader(f,delimiter = '\t')
        i = 0
        pts_img = np.zeros((4,2))
        M = np.zeros((3,3))
        for line in c:
            if i == 0:
                p = [float(x) for x in line if x!='']
            elif i >= 0 and i <=4:
                pts_img[i-1,:] = line
            else:
                M[i-5,:] = line
            i+=1
    return np.array(p),np.array(pts_img),np.matrix(M)

fname = get_parameters()
p,pts_img,M = get_positions(fname)
