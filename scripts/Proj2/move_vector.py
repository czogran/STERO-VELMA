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
 

 
def move_vector(velma, vector):
    print "Moving right wrist to pose defined in world frame..."
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), vector)
    if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(13)
    if velma.waitForEffectorRight() != 0:
        exitError(14)
    rospy.sleep(0.5)
    #print "Calculating difference between desiread and reached pose..."
    #T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Tr"), 1.0)
    #print T_B_T_diff
    #if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
     #   exitError(15)
    
def move_rotation(velma, rotation):
    print "Moving right wrist to pose defined in world frame..."
    T_B_Trd = PyKDL.Frame(rotation, PyKDL.Vector())
    if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(13)
    if velma.waitForEffectorRight() != 0:
        exitError(14)
    rospy.sleep(0.5)

def move_frame(velma, frame,time_wrist=6,frame_tool=None,time_tool=None):
    print "Moving right wrist to pose defined in world frame..."
  #  if not velma.moveCartImpRight([frame], [3.0], frame_tool_vector, time_tool_vector, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
    twist=PyKDL.Twist(PyKDL.Vector(0.1,0.1,0.1),PyKDL.Vector(0.1,0.1,0.1))
    wr=PyKDL.Wrench(PyKDL.Vector(80,80,500), PyKDL.Vector(120,120,120))
    if not velma.moveCartImpRight([frame], [time_wrist], frame_tool, time_tool, [wr], [time_wrist], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5,path_tol=twist):
        print "wait for three"
        exitError(13)
    try:
        if velma.waitForEffectorRight() != 0:
            print "wait for effector right"
            return True
    except:
        rospy.sleep(0.5)
    
def go_to_current_position(velma):
   print "go to current"
   if not velma.moveCartImpRightCurrentPos(start_time=0.2):
		exitError(8)
   if velma.waitForEffectorRight() != 0:
	    exitError(9)
   #rospy.sleep(0.1)
