#!/usr/bin/env python

# This is a code example for Cosi119a - Autonomous Robotics
# It illustrates
# 1. how to subscribe and publish images - from the camera and to another topic
# 2. how to use opencv to do a variety of useful transformations
# 3. how to use dynamic_reconfigure to change parameters and see the effect

""" 
NOTES: Make sure the camera is on. Verify that you see the images either with rviz or rqt_image_view (https://wiki.ros.org/rqt_image_view). This program will take that image and subject it to several transformations and publish the transformed images on other topics. Also run rqt_reconfigure (https://wiki.ros.org/rqt_reconfigure) which will give you a simple UI where you can play with parameters to see what works best. You will have to read the code to see what it does in more detail. I will add features to it as they come up.
 """

import rospy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from std_msgs.msg import String
import numpy as np
import cv2 as cv
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from prrexamples.cfg import CvexampleConfig as ConfigType

class CvExample():

    def cv_callback(self, msg):
        if (self.param_ready):
            rospy.loginfo("Image callback")
            self.rgb_image = CvBridge().compressed_imgmsg_to_cv2(msg)
            self.hsv_image = cv.cvtColor(self.rgb_image, cv.COLOR_BGR2HSV)
            self.create_masked_image()
            self.create_grey_image()
            self.create_centroid_image()
            self.create_blurred_image()
            self.create_contours()

    def create_masked_image(self):
        # range of colors, found by trial and error
        lower_color_bound = np.array([self.config.lb_h, self.config.lb_s, self.config.lb_v])
        upper_color_bound = np.array([self.config.ub_h, self.config.ub_s, self.config.ub_v])

        # find pixels in range bounded by BGR color bounds
        self.mask = cv.inRange(self.hsv_image, lower_color_bound, upper_color_bound)

        # find pixels that are in both mask AND original img
        self.masked_hsv_img = cv.bitwise_and(self.hsv_image, self.hsv_image, mask=self.mask)
        self.masked_rgb_image = cv.cvtColor(self.masked_hsv_img, cv.COLOR_HSV2BGR)

        masked_msg = CvBridge().cv2_to_compressed_imgmsg(self.masked_rgb_image)
        self.masked_pub.publish(masked_msg)
    
    def create_grey_image(self):
        self.grey_image = cv.cvtColor(self.rgb_image, cv.COLOR_RGB2GRAY) 
        self.grey_masked_image = cv.cvtColor(self.masked_rgb_image, cv.COLOR_RGB2GRAY) 
        grey_masked_msg = CvBridge().cv2_to_compressed_imgmsg(self.grey_masked_image)
        self.grayed_pub.publish(grey_masked_msg)

    def create_centroid_image(self):
        self.centroid_image = self.rgb_image.copy()
        M = cv.moments(self.grey_masked_image)
        if M['m00'] > 0:
             cx = int(M['m10']/M['m00'])
             cy = int(M['m01']/M['m00'])
             cv.circle(self.centroid_image, (cx, cy), 20, (0,255,255), -1)
        centroid_msg = CvBridge().cv2_to_compressed_imgmsg(self.centroid_image)
        self.centroid_pub.publish(centroid_msg)

    def create_blurred_image(self):
        # Now blurr
        self.blurred_image = cv.GaussianBlur(self.grey_masked_image, (self.config.blurr*2+1, self.config.blurr*2+1), 0)  # blur image with a 5x5 kernel
        blurred_msg = CvBridge().cv2_to_compressed_imgmsg(self.blurred_image)
        self.blurred_pub.publish(blurred_msg)

    def create_contours(self):
        # # Lets try countours
        ret, thresh = cv.threshold(
            self.blurred_image, 0, 255, cv.THRESH_BINARY_INV)  # create an threshold
        contours, hierachy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        self.contour_image = cv.drawContours(thresh, contours, -1, (255, 255, 255), 20)
        contour_msg = CvBridge().cv2_to_compressed_imgmsg(self.contour_image)
        self.contour_pub.publish(contour_msg)

    def dynamic_cb(self, config, level):
        rospy.loginfo("Dynamic Config callback {lb_h}:{lb_s}:{lb_v} {ub_h}:{ub_s}:{ub_v}".format(**config))
        self.config = config
        self.param_ready = True
        return config

    def __init__(self):
        self.param_ready = False
        self.cam_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.cv_callback)
        self.masked_pub = rospy.Publisher("/cvexample/masked/compressed", CompressedImage, queue_size=1)
        self.grayed_pub = rospy.Publisher("/cvexample/grayed/compressed", CompressedImage, queue_size=1)
        self.blurred_pub = rospy.Publisher("/cvexample/blurred/compressed", CompressedImage, queue_size=1)
        self.contour_pub = rospy.Publisher("/cvexample/contour/compressed", CompressedImage, queue_size=1)
        self.centroid_pub = rospy.Publisher("/cvexample/centroid/compressed", CompressedImage, queue_size=1)
        self.dynamic = DynamicReconfigureServer(ConfigType, self.dynamic_cb)
        rospy.loginfo("Initialized")

# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("cvexample")
    # Go to class functions that do all the heavy lifting.
    try:
        CvExample()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()

