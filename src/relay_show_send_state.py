#! /usr/bin/env python
import sys 
sys.path.append("/home/cby/underactuated/") 

import math
import numpy as np
import matplotlib.pyplot as plt
from pydrake.geometry import ConnectDrakeVisualizer
from pydrake.all import (DiagramBuilder, DirectCollocation, MultibodyPlant,
                         MultibodyPositionToGeometryPose, Parser,
                         PiecewisePolynomial, PlanarSceneGraphVisualizer,
                         SceneGraph, Simulator, Solve, TrajectorySource)
from underactuated import FindResource


import rospy
#from gazebo_msgs.msg import LinkStates
from drake_learning1.msg import ourbot_state
from drake_learning1.msg import ourbot_input

rospy.init_node("get_link_state")
#import tf

def callback(msg):
	# print(msg.pose[2].orientation)
	#(r, p, y) = tf.transformations.euler_from_quaternion([msg.pose[2].orientation.x, msg.pose[2].orientation.y, msg.pose[2].orientation.z, msg.pose[2].orientation.w])
	#print(p)
	#print(msg.pose[2].position.x)
	#print('常量 PI 的值近似为： {}。'.format(math.pi))
    drake_input = ourbot_input()
	drake_state = ourbot_state()
	rate = rospy.Rate(10) 
	
	
	
	
    global i
    global sum_x
    """for i in range(10):
	    current_x[i] = msg.pose[2].position.x
		sum_x = sum_x +  current_x[i]
		print(current_x[i])
	averagex_10=sum_x/10"""
    statex=msg.pose[2].position.x
    print('current x state is :{}'.format(statex))
    if i<9:
        i=i+1
        sum_x = sum_x +statex
    elif i==9:
        i=0
        averagex_10=(sum_x+statex)/10
        sum_x=0
        print('average current x state is :{}'.format(averagex_10))#1000HZ->100HZ
        cartpole_msg.current_x = averagex_10
        x_pub.publish(statex)
	#rospy.sleep(1.)

if __name__ == '__main__':
    #rospy.init_node('xpublisher', anonymous=True)
    i=0
    sum_x=0
	#x_pub_rhip = rospy.Publisher('/curretx0_gazebo', Float64, queue_size=100)
	#x_pub_lhip = rospy.Publisher('/curretx0_gazebo', Float64, queue_size=100)
	#x_pub_rknee = rospy.Publisher('/curretx0_gazebo', Float64, queue_size=100)
    #x_pub_lknee = rospy.Publisher('/curretx0_gazebo', Float64, queue_size=100)
    drake_lefthip_torque_pub = rospy.Publisher('/left_hip_position_controller/command', Float64, queue_size=100)
    drake_righthip_torque_pub = rospy.Publisher('/right_hip_position_controller/command', Float64, queue_size=100)
    drake_leftknee_torque_pub = rospy.Publisher('/left_knee_position_controller/command', Float64, queue_size=100)
    drake_rightknee_torque_pub = rospy.Publisher('/right_knee_position_controller/command', Float64, queue_size=100)
    sub_state = rospy.Subscriber("/planned_ourbot_input",ourbot_state,callback, queue_size=1, buff_size=100)#1000HZ
	sub_input = rospy.Subscriber("/planned_ourbot_state",ourbot_input,callback, queue_size=1, buff_size=100)#1000HZ
    rospy.spin()