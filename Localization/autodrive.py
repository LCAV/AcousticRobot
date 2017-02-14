
# coding: utf-8

import math
import numpy as np
import csv

def area(array):
#array should be like (x,y)
    yaxis = (0,1)
    angle = (array[0]*yaxis[0]+array[1]*yaxis[1])/(math.sqrt(math.pow(array[0],2)+math.pow(array[1],2)))
    THETA = np.arccos(angle)
    if array[0] > 0:
        THETA = THETA*180/math.pi
    else:
        THETA = -THETA*180/math.pi
    
    return THETA

def read_file(inputfile):
    points_array=[]
    with open(inputfile) as f:
        c=csv.reader(f)
        for line in c :
            points_array.extend(line)
    return points_array
def get_para(inputfile):
    c=read_file(inputfile)
    points=[]
    for i in range(0,len(c),2):
        points.append((int(c[i]),int(c[i+1])))

    array=[]
    for i in range(0, len(points)-1):
        array.append((points[i+1][0]-points[i][0],points[i+1][1]-points[i][1]))

    d1=math.sqrt(math.pow(points[1][0]-points[0][0],2)+math.pow(points[1][1]-points[0][1],2))
    d=[d1]
    for i in range(0,len(points)-2):
        d.append(math.sqrt(math.pow(points[i+2][0]-points[i+1][0],2)+math.pow(points[i+2][1]-points[i+1][1],2)))

    theta1=np.arctan(points[1][1]/points[1][0])*180/math.pi
    theta=[theta1]
    for i in range(0,len(array)-1):
        cos=(array[i][0]*array[i+1][0]+array[i][1]*array[i+1][1])/d[i]/d[i+1]
        theta.append(np.arccos(cos)*180/math.pi)

    ref_theta=[]
    for i in range(0,len(array)):
        ref_theta.append(area(array[i]))

    angle=[0]*len(theta)
    for i in range(0,len(theta)):
        if i ==0:
            angle[i]=(round(theta[i],2),'r')
        elif round(ref_theta[i]-ref_theta[i-1],3)==round(theta[i],3):
            angle[i]=(round(theta[i],2),'r')   
        elif round(-ref_theta[i]+ref_theta[i-1],3)==round(theta[i],3):
            angle[i]=(round(theta[i],2),'l')
        elif round(-ref_theta[i]+360+ref_theta[i-1],3)==round(theta[i],3):
            angle[i]=(round(theta[i],2),'l')
        elif round(ref_theta[i]-360-ref_theta[i-1],3)==round(theta[i],3):
            angle[i]=(round(theta[i],2),'r')

    return angle,d




angle,distance=get_para('autocommand.txt')

