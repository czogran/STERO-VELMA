#!/usr/bin/env python
 
 
import roslib;     roslib.load_manifest('velma_task_cs_ros_interface')
 
import rospy
import math
import PyKDL
 
from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError
 
 
q_map_pose = {'torso_0_joint':0, 'right_arm_0_joint':1.60, 'right_arm_1_joint':-0.99,
         'right_arm_2_joint':-1.85, 'right_arm_3_joint':-1.8, 'right_arm_4_joint':-2.70, 'right_arm_5_joint':-1.9,
         'right_arm_6_joint':0.11,'head_tllt_JoInt':-0.63}

q_map_goal = {'torso_0_joint':0, 'right_arm_0_joint':-0.46, 'right_arm_1_joint':-1.2,
         'right_arm_2_joint':-1.54, 'right_arm_3_joint':-1.96, 'right_arm_4_joint':-2.23, 'right_arm_5_joint':-1.63,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
         
q_map_goal1 = {'torso_0_joint':0, 'right_arm_0_joint':-0.46, 'right_arm_1_joint':-1.2,
         'right_arm_2_joint':-1.54, 'right_arm_3_joint':-1.96, 'right_arm_4_joint':-2.23, 'right_arm_5_joint':-1.63,
         'right_arm_6_joint':1.3, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }


q_map_starting = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }


q_map_torso = {'torso_0_joint':0}
q_map_torso1 = {'torso_0_joint':1}
 
def exitError(code):
     if code == 0:
         print "OK"
         exit(0)
     print "ERROR:", code
     exit(code) 
 

 
def move_joint(velma,planner, pose):

    
    print "Moving to valid position, using planned trajectory."
    goal_constraint_1 = qMapToConstraints(pose, 0.01, group=velma.getJointGroup("impedance_joints"))
    for i in range(5):
         print 'move_joint iteration ' + str(i)
         rospy.sleep(0.5)
         js = velma.getLastJointState()
         print "Planning (try", i, ")..."
         traj = planner.plan(js[1], [goal_constraint_1], "impedance_joints", num_planning_attempts=10, max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
         if traj == None:
             continue
         print "Executing trajectory..."
         #print traj
         if not velma.moveJointTraj(traj, start_time=0.5, position_tol=10.0/180.0 * math.pi, velocity_tol=10.0/180.0*math.pi):
            print 'Error in moveJointTraj'
            exitError(9)
         if velma.waitForJoint() == 0:
            break
         else:
             print "The trajectory could not be completed, retrying..."
             continue

def move_joint_noplan(velma, pose):
    print "Moving to valid position,no plan."
    velma.moveJoint(pose, 3, start_time=0.5, position_tol=0.1, velocity_tol=0)
    error = velma.waitForJoint()
    rospy.sleep(0.5)
   
    
