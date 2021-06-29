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
from sensor_msgs.msg import JointState
#rospy.init_node("get_link_state")
#import tf

def callback(msg):
    lh_u=Float64()
    rh_u=Float64()
    lk_u=Float64()
    rk_u=Float64()
    lh_s=Float64()
    rh_s=Float64()
    lk_s=Float64()
    rk_s=Float64()

    lh_u.data=msg.effort[0]
    lk_u.data=msg.effort[1]
    rh_u.data=msg.effort[2]
    rk_u.data=msg.effort[3]
    lh_s.data=msg.position[0]
    lk_s.data=msg.position[1]
    rh_s.data=msg.position[2]
    rk_s.data=msg.position[3]

    lh_u_pub.publish(lh_u)
    lk_u_pub.publish(lk_u)
    rh_u_pub.publish(rh_u)
    rk_u_pub.publish(rk_u)
    lh_s_pub.publish(lh_s)
    lk_s_pub.publish(lk_s)
    rh_s_pub.publish(rh_s)
    rk_s_pub.publish(rk_s)

"""
    times = np.linspace(0, 1, 100)	
    fig, ax = plt.subplots(2,1,figsize=(8,8))
    plt.subplots_adjust(wspace =0, hspace =0.5)
    #plt.tight_layout(3)#adjust total space
    ax[0].set_title('left leg state from gazebo')
    ax[0].plot(times,lh_s, linewidth=2, color='b', linestyle='-')
    ax[0].set_xlabel("times")
    ax[0].set_ylabel("left state");
    ax[0].plot(times, lk_s,color='r',linewidth=2,linestyle='--')
    ax[0].legend(('lh_state','lk_state'));

    ax[1].set_title('right leg state from gazebo')
    ax[1].plot(times,rh_s, color='b', linestyle='-')
    ax[1].plot(times,rk_s, color='r',linewidth=2,linestyle='--')
    ax[1].legend(('rh_state','rk_state'));
    ax[1].set_xlabel("times")
    ax[1].set_ylabel("right state")
	
    ax[2].set_title('left leg input from gazebo')
    ax[2].plot(times,lh_u, color='b', linestyle='-')
    ax[2].plot(times,lk_u, color='r',linewidth=2,linestyle='--')
    ax[2].legend(('lh_input','lk_input'));
    ax[2].set_xlabel("times")
    ax[2].set_ylabel("left leg input")

    ax[3].set_title('right leg input from gazebo')
    ax[3].plot(times,rh_u, color='b', linestyle='-')
    ax[3].plot(times,rk_u, color='r',linewidth=2,linestyle='--')
    ax[3].legend(('rh_input','rk_input'));
    ax[3].set_xlabel("times")
    ax[3].set_ylabel("right leg input")
    plt.show()"""

if __name__ == '__main__':
    rospy.init_node('relay_show_send_input', anonymous=True)
    i=0
    sum_x=0
    lh_u_pub = rospy.Publisher('/lh_u_pub', Float64, queue_size=100)
    lk_u_pub = rospy.Publisher('/lk_u_pub', Float64, queue_size=100)
    rh_u_pub = rospy.Publisher('/rh_u_pub', Float64, queue_size=100)
    rk_u_pub = rospy.Publisher('/rk_u_pub', Float64, queue_size=100)
    lh_s_pub = rospy.Publisher('/lh_s_pub', Float64, queue_size=100)
    lk_s_pub = rospy.Publisher('/lk_s_pub', Float64, queue_size=100)
    rh_s_pub = rospy.Publisher('/rh_s_pub', Float64, queue_size=100)
    rk_s_pub = rospy.Publisher('/rk_s_pub', Float64, queue_size=100)
    sub = rospy.Subscriber("/joint_states",JointState,callback, queue_size=1, buff_size=100)#50HZ
     #service

    rospy.loginfo("listening")
    #sub_state = rospy.Subscriber("/planned_ourbot_state",ourbot_state,callback, queue_size=1, buff_size=100)#1000HZ
    rospy.spin()