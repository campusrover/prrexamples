#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from std_msgs.msg import String
import numpy as np
import cv2 as cv

rospy.init_node('cvexample')
BRIDGE = CvBridge()

def cv_callback(msg):
   cv_image = BRIDGE.imgmsg_to_cv2(msg)
   do_stuff(cv_image)

def do_stuff(img):
   # get hsv image from opencv
   hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV) 

   # range of colors, found by trial and error
   lower_color_bound = np.array([50,1,1]) 
   upper_color_bound = np.array([90,255,255]) 
  
   # find pixels in range bounded by BGR color bounds
   mask = cv.inRange(hsv, lower_color_bound, upper_color_bound)

   # find pixels that are in both mask AND original img
   masked_img = cv.bitwise_and(img, img, mask=mask)
   ros_img = BRIDGE.cv2_to_imgmsg(masked_img)
   rgb_pub.publish(ros_img)
   # hsv_img = BRIDGE.cv2_to_imgmsg(hsv)
   # hsv_pub.publish(hsv_img)    

# subscriber/publishers
cam_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, cv_callback)
rgb_pub = rospy.Publisher('/camera/rgb/masked', Image, queue_size=1)
# hsv_pub = rospy.Publisher('/camera/rgb/hsv_image', Image, queue_size=1)

# control loop
while not rospy.is_shutdown():
    rospy.sleep(10)
