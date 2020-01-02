#!/usr/bin/env python
 
 
import roslib;     roslib.load_manifest('velma_task_cs_ros_interface')
 
import rospy
import math
import PyKDL
 
from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError
 
def exitError(code):
     if code == 0:
         print "OK"
         exit(0)
     print "ERROR:", code
     exit(code) 
 
def grip(velma, angle):

    rad_angle=angle/180.0*math.pi
    dest_q = [rad_angle,rad_angle, rad_angle, 0]
    
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
         exitError(8)
    rospy.sleep(0.5)
    #if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
     #    exitError(9)



   
    
