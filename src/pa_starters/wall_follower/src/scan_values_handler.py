#!/usr/bin/env python

#This processes all of the scan values


import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, Float32
from constants import *

#Process all the data from the LIDAR
def cb(msg):
    #Determine state
    pub_state.publish(s)
    #CALCULATE AND PUBLISH ANYTHING ELSE FOR THE PID



#Init node
rospy.init_node('scan_values_handler')

#Subscriber for LIDAR
sub = rospy.Subscriber('scan', LaserScan, cb)

#Publishers
pub_state = rospy.Publisher('state', Int16, queue_size = 1)
#THINK OF WHAT INFO TO PUBLISH TO THE PID

#Rate object
rate = rospy.Rate(10)

#Keep the node running
while not rospy.is_shutdown():
    rate.sleep() 