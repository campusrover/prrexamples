#!/usr/bin/env python

# Starter Code for New PA2
# Based on code from August Soderberg
import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np

def scan_cb(msg):
    global ranges
    ranges = np.array(msg.ranges)

def odom_cb(msg):
    global pose
    pose = msg.pose

#Set up node and pubs/subs

rospy.init_node('pilot')
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_cb)
odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

pose = None
rate = rospy.Rate(2)

start_time = rospy.Time.now()
while pose == None:
    print("Waiting for simulated robot")

start_pose = pose

while not rospy.is_shutdown():
    if rospy.Time.now() - start_time < rospy.Duration(secs=30):
        print(pose.pose.position.x)
        print(">>>>>>>>>>> roomba is vaccuming")
    else:
        print("<<<<<<<<<< roomba returns to start")
        break
    rate.sleep()
