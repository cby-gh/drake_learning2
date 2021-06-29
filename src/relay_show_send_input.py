#! /usr/bin/env python
import sys 
sys.path.append("/home/cby/underactuated/") 
from std_msgs.msg import Float64

import math
import numpy as np
import matplotlib.pyplot as plt
from pydrake.geometry import ConnectDrakeVisualizer
from pydrake.all import (DiagramBuilder, DirectCollocation, MultibodyPlant,
                         MultibodyPositionToGeometryPose, Parser,
                         PiecewisePolynomial, PlanarSceneGraphVisualizer,
                         SceneGraph, Simulator, Solve, TrajectorySource)
from underactuated import FindResource
from drake_learning1.srv import ourbot_inputsrv#, PersonSrvResponse

import rospy
#from gazebo_msgs.msg import LinkStates
from drake_learning1.msg import ourbot_state
from drake_learning1.msg import ourbot_input

#rospy.init_node("get_link_state")
#import tf

def callback(msg):
	# print(msg.pose[2].orientation)
	#(r, p, y) = tf.transformations.euler_from_quaternion([msg.pose[2].orientation.x, msg.pose[2].orientation.y, msg.pose[2].orientation.z, msg.pose[2].orientation.w])
	#print(p)
	#print(msg.pose[2].position.x)
	#print('常量 PI 的值近似为： {}。'.format(math.pi))
    rospy.loginfo("received")		
    drake_input = ourbot_input()
    drake_input = msg
    lh = []
    rh = []
    lk = []
    rk = []
    lh_state = []
    rh_state = []
    lk_state = []
    rk_state = []
    print(type(lk))
    i=0
    for i in range(100):
        lh.append(1*drake_input.hip_lpin_torque[i])#1.5 or 1.3 is for ourbot, 1.0 is the planarwalker3
        #drake_leftthip_torque_reference.publish(drake_input.hip_lpin_torque[i])
        print("lh data received from drake solver:{}".format(lh[i]))
        rh.append(1*drake_input.hip_rpin_torque[i])
        #drake_righthip_torque_reference.publish(drake_input.hip_rpin_torque[i])
        #print("rh data received from drake solver:{}".format(rh[i]))
        lk.append(1*drake_input.left_knee_pin_torque[i])
        #drake_lefttknee_torque_reference.publish(drake_input.left_knee_pin_torque[i])
        #print("lk data received from drake solver:{}".format(lk[i]))
        rk.append(1*drake_input.right_knee_pin_torque[i])
        #print("rk data received from drake solver:{}".format(rk[i]))
        #drake_rightknee_torque_reference.publish(drake_input.right_knee_pin_torque[i])
        lh_state.append(1.0*drake_input.hip_lpin_state[i])
        rh_state.append(1.0*drake_input.hip_rpin_state[i])
        lk_state.append(1.0*drake_input.left_knee_pin_state[i])
        rk_state.append(1.0*drake_input.right_knee_pin_state[i])	
        #print("231312321213123131331")
    times = np.linspace(0, 1, 100)	
    i=0
    print(type(lk))
    print("lenth received:{}".format(len(lh)))
    cmd_lh = 0.0
    cmd_rh = 0.0
    cmd_lk= 0.0
    cmd_rk = 0.0
    i = 0
		
	#drake_state = ourbot_state()
    rate = rospy.Rate(100) 
    while not rospy.is_shutdown():
    #while i<=99:
        drake_leftthip_torque_reference.publish(drake_input.hip_lpin_torque[i])
        drake_righthip_torque_reference.publish(drake_input.hip_rpin_torque[i])
        drake_lefttknee_torque_reference.publish(drake_input.left_knee_pin_torque[i])
        drake_rightknee_torque_reference.publish(drake_input.right_knee_pin_torque[i])
        #print(len(drake_input.hip_lpin_torque))
        #双腿调换才是对的
        cmd_lh = 1.0*lh[i]
        cmd_rh = 1.0*rh[i]
        cmd_lk = 1.0*lk[i]
        cmd_rk = 1.0*rk[i]
        #双腿不调换
       # cmd_lh = 1.0*drake_input.hip_lpin_torque[i]
        #cmd_rh = 1.0*drake_input.hip_rpin_torque[i]
        #cmd_lk = 1.0*drake_input.left_knee_pin_torque[i]
        #cmd_rk = 1.0*drake_input.right_knee_pin_torque[i]
        drake_lefthip_torque_pub.publish(cmd_lh)
        drake_righthip_torque_pub.publish(cmd_rh)
        drake_leftknee_torque_pub.publish(cmd_lk)
        drake_rightknee_torque_pub.publish(cmd_rk)
        print("drake_leftknee_torque_pub:{}".format(cmd_lk))
        temp1 = lh[i]
        temp2 = lk[i]
        lh[i] = rh[i]
        lk[i] = rk[i]
        rh[i] = temp1
        rk[i] = temp2		
       # print("\t:{}".format(i))
        if i ==99:
            i=0
        elif i<99:
            i = i+1
        # 发布消息
    #while not rospy.is_shutdown():
    #    cmd_lh = 0
     #   cmd_rh = 0
    #    cmd_lk = 0
    #    cmd_rk = 0
   #     drake_lefthip_torque_pub.publish(cmd_lh)
   #     drake_righthip_torque_pub.publish(cmd_rh)
   #     drake_leftknee_torque_pub.publish(cmd_lk)
    #    drake_rightknee_torque_pub.publish(cmd_rk)
       # rospy.loginfo("Publsh message")		
#    	rospy.loginfo("Publsh person message[%s, %d, %d]", 
#				person_msg.name, person_msg.age, person_msg.sex)
        rate.sleep()	
        #state in one plot and input in one plot
    drake_lefthip_torque_pub.publish(0.0)
    drake_righthip_torque_pub.publish(0.0)
    drake_leftknee_torque_pub.publish(0.0)
    drake_rightknee_torque_pub.publish(0.0)
    fig, ax = plt.subplots(4,1,figsize=(8,8))
    plt.subplots_adjust(wspace =0, hspace =0.5)
    #plt.tight_layout(3)#adjust total space
    ax[0].set_title('left leg state from drake')
    ax[0].plot(times,lh_state, linewidth=2, color='b', linestyle='-')
    ax[0].set_xlabel("times")
    ax[0].set_ylabel("left state");
    ax[0].plot(times, lk_state,color='r',linewidth=2,linestyle='--')
    ax[0].legend(('lh_state','lk_state'));

    ax[1].set_title('right leg state from drake')
    ax[1].plot(times,rh_state, color='b', linestyle='-')
    ax[1].plot(times,rk_state, color='r',linewidth=2,linestyle='--')
    ax[1].legend(('rh_state','rk_state'));
    ax[1].set_xlabel("times")
    ax[1].set_ylabel("right state")
	
    ax[2].set_title('left leg input from drake')
    ax[2].plot(times,lh, color='b', linestyle='-')
    ax[2].plot(times,lk, color='r',linewidth=2,linestyle='--')
    ax[2].legend(('lh_input','lk_input'));
    ax[2].set_xlabel("times")
    ax[2].set_ylabel("left leg input")

    ax[3].set_title('right leg input from drake')
    ax[3].plot(times,rh, color='b', linestyle='-')
    ax[3].plot(times,rk, color='r',linewidth=2,linestyle='--')
    ax[3].legend(('rh_input','rk_input'));
    ax[3].set_xlabel("times")
    ax[3].set_ylabel("right leg input")
    plt.show()

if __name__ == '__main__':
    rospy.init_node('relay_show_send_input', anonymous=True)
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
	
    drake_rightknee_torque_reference = rospy.Publisher('/right_knee_reference', Float64, queue_size=100)
    drake_lefttknee_torque_reference = rospy.Publisher('/left_knee_reference', Float64, queue_size=100)
    drake_righthip_torque_reference = rospy.Publisher('/right_hip_reference', Float64, queue_size=100)
    drake_leftthip_torque_reference = rospy.Publisher('/left_hip_reference', Float64, queue_size=100)
    #sub_input = rospy.Subscriber("/planned_ourbot_input",ourbot_input,callback, queue_size=1, buff_size=100)#1000HZ
     #service
    sub_input = rospy.Service('/Planned_trajectory', ourbot_inputsrv, callback)

    rospy.loginfo("listening")
    #sub_state = rospy.Subscriber("/planned_ourbot_state",ourbot_state,callback, queue_size=1, buff_size=100)#1000HZ
    rospy.spin()