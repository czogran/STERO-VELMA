#!/usr/bin/env python


import roslib;     roslib.load_manifest('velma_task_cs_ros_interface')
 
import rospy
import math
import PyKDL
 
from velma_common.velma_interface import *
from control_msgs.msg import FollowJointTrajectoryResult
 
def exitError(code):
     if code == 0:
         print "OK"
         exit(0)
     print "ERROR:", code
     exit(code)
 
if __name__ == "__main__":
     # define some configurations
 
     # every joint in position 0
     q_map_0 = {'torso_0_joint':0, 'right_arm_0_joint':0, 'right_arm_1_joint':0,
         'right_arm_2_joint':0, 'right_arm_3_joint':0, 'right_arm_4_joint':0, 'right_arm_5_joint':0,
         'right_arm_6_joint':0, 'left_arm_0_joint':0, 'left_arm_1_joint':0, 'left_arm_2_joint':0,
         'left_arm_3_joint':0, 'left_arm_4_joint':0, 'left_arm_5_joint':0, 'left_arm_6_joint':0 }
 
     # starting position
     q_map_starting = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
 
     # goal position
     q_map_goal = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':-1.25, 'right_arm_3_joint':1.57, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
 
     # intermediate position
     q_map_intermediate = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.6,
         'right_arm_2_joint':-1.25, 'right_arm_3_joint':-0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
 
     rospy.init_node('test_jimp')
 
     rospy.sleep(0.5)
 
     print "This test/tutorial executes simple motions"\
         " in joint impedance mode. Planning is not used"\
         " in this example.\n"
 
     print "Running python interface for Velma..."
     velma = VelmaInterface()
     print "Waiting for VelmaInterface initialization..."
     if not velma.waitForInit(timeout_s=10.0):
         print "Could not initialize VelmaInterface\n"
         exitError(1)
     print "Initialization ok!\n"
 
     print "Motors must be enabled every time after the robot enters safe state."
     print "If the motors are already enabled, enabling them has no effect."
     print "Enabling motors..."
     if velma.enableMotors() != 0:
         exitError(2)
 
     rospy.sleep(0.5)
 
     diag = velma.getCoreCsDiag()
     if not diag.motorsReady():
         print "Motors must be homed and ready to use for this test."
         exitError(1)
 
     print "Switch to jnt_imp mode (no trajectory)..."
     velma.moveJointImpToCurrentPos(start_time=0.5)
     error = velma.waitForJoint()
     if error != 0:
         print "The action should have ended without error, but the error code is", error
         exitError(3)
 
 
     print "Moving to position 0 "
    # velma.moveJoint(q_map_starting, 5, start_time=0.5, position_tol=0.1, velocity_tol=0)
     #error = velma.waitForJoint()
   
 
    
    
     print "Reset tools for both arms..."

     print "Switch to cart_imp mode (no trajectory)..."
     if not velma.moveCartImpRightCurrentPos(start_time=0.2):
         exitError(8)
     if velma.waitForEffectorRight() != 0:
         exitError(9)
 
     rospy.sleep(0.5)
 
     diag = velma.getCoreCsDiag()
     if not diag.inStateCartImp():
         print "The core_cs should be in cart_imp state, but it is not"
         exitError(3)
 
     rospy.sleep(0.5)
     print "Moving right wrist to pose defined in world frame..."
     figure_start = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.0 , 0.0 , 0.0 , 0 ), PyKDL.Vector( 0, -0.7, 1 ))
     vector=PyKDL.Vector(0.05, 0.05, 0)
     vectorZ=PyKDL.Vector(0.05, 0.05, 0.05)
     
     rad_angle=90/180.0*math.pi
     dest_q = [rad_angle,rad_angle, rad_angle, 0]
    
     velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
     if velma.waitForHandRight() != 0:
         exitError(8)
     rospy.sleep(0.5)
    
     poses=[]
     #start point
     poses.append(PyKDL.Frame(PyKDL.Rotation.Quaternion(0.0 , 0.0 , 0.0 , 0 ), PyKDL.Vector( 0, -0.7, 1)))
     
     times=[]
     times.append(5.0)
     size=30
     t=30/size
     for i in range(size):
        vector=PyKDL.Frame(PyKDL.Rotation.RotZ(2*math.pi/size))*vector
        vectorZ=PyKDL.Frame(PyKDL.Rotation.RotX(2*math.pi/size))*vectorZ
        poses.append(PyKDL.Frame(PyKDL.Rotation.Quaternion(0.0, 0.0 , 0.0 , 0 ), figure_start.p+vector+vectorZ))
        poses[i+1].p=PyKDL.Frame(PyKDL.Rotation.RotX(2/6*math.pi))*poses[i+1].p
        #t=6+i*3.0
        times.append(t)
        print   "  "
        print i
        
        print times[i+1]
        print poses[i+1]
        print vector
     
     #comeback to first position
     poses.append(poses[1])
     times.append(t)

     


     print "all of poses"
     print poses
     #if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
     print "  "

    
     #if not velma.moveCartImpRight(poses, times,None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
      #    exitEror(8)
     for i in range(size+2):
         print "position: {}".format(i)
         print poses[i]
         if not velma.moveCartImpRight([poses[i]], [times[i]],None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
             exitEror(8)
         if velma.waitForEffectorRight() != 0:
             exitError(9)
         rospy.sleep(0.5)
         
     
     
     
     
     
