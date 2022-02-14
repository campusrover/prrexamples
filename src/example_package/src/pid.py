#!/usr/bin/env python

#This is a PID controller to determine twist values

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32

#Linear speed of the robot
LINEAR_SPEED = 0.2
#Angular speed of the robot
ANGULAR_SPEED = 0.3

#Multipliers used to tune the PID controller
#Proportional constant
P_CONSTANT = 0
#Integral constant
I_CONSTANT = 0
#Derivative constant
D_CONSTANT = 0

def scan_handler_cb(msg):
    global nearest_distance
    nearest_distance = msg.data

#Init node
rospy.init_node('pid')

#Create publisher for suggested twist objects
pub = rospy.Publisher('twist', Twist, queue_size = 1)

sub = rospy.Subscriber('nearest_value', Float32, scan_handler_cb)

# Inital value
nearest_distance = 0

#Twist and rate object
t = Twist()
rate = rospy.Rate(10)



while not rospy.is_shutdown():
    #calculate p component
    p_component = 0
    #calculate d component
    d_component = 0
    #calculate i component
    i_component = 0
    #Add them all together, multiplied by their respective tuning values, and multiply everything
    #by the angular velocity
    t.angular.z = ANGULAR_SPEED * (P_CONSTANT * p_component + D_CONSTANT * d_component + I_CONSTANT * i_component)
    t.linear.x = LINEAR_SPEED
    #Publish the twist to the driver
    pub.publish(t)
    rate.sleep() 