#!/usr/bin/env python

#This processes all of the scan values


import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, Float32

# This about how to not hard code this to make code more modular.
# What if you scan values contained 720 items?
min_relevant_scan_angle = 330
max_relevant_scan_angle = 30

#Process all the data from the LIDAR
def cb(msg):
    nearest_value = Float32()
    #This for loop is useless but it gives you an idea of how you would actually process the scan values
    for value in msg.ranges:
        nearest_value.data = value
    pub_nearest.publish(nearest_value)

#Init node
rospy.init_node('scan_values_handler')

#Subscriber for LIDAR
sub = rospy.Subscriber('scan', LaserScan, cb)

#Publishers
pub_nearest = rospy.Publisher('nearest_value', Float32, queue_size = 1)
#THINK OF WHAT INFO TO PUBLISH TO THE PID

#Rate object
rate = rospy.Rate(10)

#Keep the node running
while not rospy.is_shutdown():
    rate.sleep() 