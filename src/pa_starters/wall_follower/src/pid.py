#!/usr/bin/env python

#This is a PID controller to determine twist values

import rospy
from state_definitions import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32

#Linear speed of the robot
LINEAR_SPEED = 0.3
#Angular speed of the robot
ANGULAR_SPEED = 3.1415926/6

#Multipliers used to tune the PID controller
#Proportional constant
P_CONSTANT = 
#Integral constant
I_CONSTANT = 
#Derivative constant
D_CONSTANT = 

#CALLBACKS FOR ANYTHING YOUR PID NEEDS TO SUBSCRIBE TO FROM scan_values_handler

#Init node
rospy.init_node('pid')

#Create publisher for suggested twist objects
pub = rospy.Publisher('twist', Twist, queue_size = 1)

#SUBSCRIBERS FOR THINGS FROM scan_values_handler YOU MIGHT WANT

#Twist and rate object
t = Twist()
rate = rospy.Rate(10)



while not rospy.is_shutdown():
    #calculate p component
    p_component = 
    #calculate d component
    d_component = 
    #calculate i component
    i_component = 
    #Add them all together, multiplied by their respective tuning values, and multiply everything
    #by the angular velocity
    t.angular.z = ANGULAR_SPEED * (P_CONSTANT * p_component + D_CONSTANT * d_component + I_CONSTANT * i_component)
    t.linear.x = LINEAR_SPEED
    #Publish the twist to the driver
    pub.publish(t)
    rate.sleep() 