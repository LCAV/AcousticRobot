# -*- coding: utf-8 -*-
"""
Created on Thu Jul  9 09:43:36 2015

@author: kiki

Test reading 


"""

def hello(name):
    """Given an object 'name', print 'Hello ' and the object."""
    print("Hello {}".format(name))

def demo(x):
    for i in range(5):
        print("i={}, x={}".format(i,x))
        x = x + 1

i = 42
if __name__ == "__main__":
    hello(i)
    demo(0);