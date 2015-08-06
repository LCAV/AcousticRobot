# -*- coding: utf-8 -*-
from __future__ import division, print_function
import numpy as np
import matplotlib.pyplot as plt
import csv

def get_positions():
    with open("positions.txt","r") as f:
        c = csv.reader(f,delimiter = '\t')
        i = 0
        r = np.zeros((4,2))
        for line in c:
            if i == 0:
                p_rel = [float(x) for x in line if x!='']
            elif i == 1:
                p = [float(x) for x in line if x!='']
            else:
                r[i-2,:] = line
            i+=1
    return np.array(p_rel),np.array(p),r


p_rel, p, r = get_positions()

