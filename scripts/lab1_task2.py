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
 
    
     # starting position
     q_map_starting = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }

     q_map_left = {'torso_0_joint':1.56, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
    
     q_map_right = {'torso_0_joint':-1.56, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
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
     velma.moveJoint(q_map_starting, 5, start_time=0.5, position_tol=0.1, velocity_tol=0)
     error = velma.waitForJoint()
   
 
    
    
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
 

     print "head straight"
     q_dest = (0, 0)
     velma.moveHead(q_dest, 5.0, start_time=0.5) 
     if velma.waitForHead() != 0:
         exitError(8)
     rospy.sleep(0.5)

     ########moving left down
     print "Moving to start"
     velma.moveJoint(q_map_left, 5, start_time=0.5, position_tol=0.1, velocity_tol=0)
     error = velma.waitForJoint()
     rospy.sleep(0.5)

     print "head left down"
     q_dest = (1.56, 1.1)
     velma.moveHead(q_dest, 5.0, start_time=0.5) 
     if velma.waitForHead() != 0:
         exitError(8)
     rospy.sleep(0.5)

     print "head left middle"
     q_dest = (0, 1.1)
     velma.moveHead(q_dest, 5.0, start_time=0.5) 
     if velma.waitForHead() != 0:
         exitError(8)
     rospy.sleep(0.5)
    
    ####### moving right down
     print "Moving right"
     velma.moveJoint(q_map_right, 5, start_time=0.5, position_tol=0.1, velocity_tol=0)
     error = velma.waitForJoint()
     rospy.sleep(0.5)


     print "head right down"
     q_dest = (-1.56, 1.1)
     velma.moveHead(q_dest, 5.0, start_time=0.5) 
     if velma.waitForHead() != 0:
         exitError(8)
     rospy.sleep(0.5)

     print "head right up"
     q_dest = (-1.56, -0.4)
     velma.moveHead(q_dest, 5.0, start_time=0.5) 
     if velma.waitForHead() != 0:
         exitError(8)
     rospy.sleep(0.5)

     print "head  middle up"
     q_dest = (0, -0.4)
     velma.moveHead(q_dest, 5.0, start_time=0.5) 
     if velma.waitForHead() != 0:
         exitError(8)
     rospy.sleep(0.5)


     #moving left up
     print "Moving left"
     velma.moveJoint(q_map_left, 5, start_time=0.5, position_tol=0.1, velocity_tol=0)
     error = velma.waitForJoint()
     rospy.sleep(0.5)

     print "head left up"
     q_dest = (1.56, -0.4)
     velma.moveHead(q_dest, 5.0, start_time=0.5) 
     if velma.waitForHead() != 0:
         exitError(8)
     rospy.sleep(0.5)


     print "head left middle"
     q_dest = (1.56, 0.3)
     velma.moveHead(q_dest, 5.0, start_time=0.5) 
     if velma.waitForHead() != 0:
         exitError(8)
     rospy.sleep(0.5)


     print "head middle middle"
     q_dest = (0, 0.3)
     velma.moveHead(q_dest, 5.0, start_time=0.5) 
     if velma.waitForHead() != 0:
         exitError(8)
     rospy.sleep(0.5)




     ####### moving right middle
     print "Moving right"
     velma.moveJoint(q_map_right, 5, start_time=0.5, position_tol=0.1, velocity_tol=0)
     error = velma.waitForJoint()
     rospy.sleep(0.5)


     print "head right middle"
     q_dest = (-1.56, 0.3)
     velma.moveHead(q_dest, 5.0, start_time=0.5) 
     if velma.waitForHead() != 0:
         exitError(8)
     rospy.sleep(0.5)

     


   
     
     
