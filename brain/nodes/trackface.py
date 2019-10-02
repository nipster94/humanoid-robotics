# -*- coding: utf-8 -*-
"""
Created on Sun Sep 29 23:07:20 2019

@author: onthe
"""

import math

def trackface(x,y,w,h):
    height = 480
    width = 640
    f = 3.2  #focal length
    e = 25.4/300 #pixel size
    
    angle_x = math.atan((x-0.5*width)/(f/e))*180/math.pi+90
    angle_y = math.atan((y-0.5*height)/(f/e))*180/math.pi+90
    
    facesize = 170
    distance = facesize*f/(w*e)
    
    return angle_x,angle_y,distance

    
    