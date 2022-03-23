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
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from prrexamples.cfg import CvexampleConfig as ConfigType


class CvExample():

    def cv_callback(self, msg):
        if (self.param_ready):
            rospy.loginfo("Image callback")
            self.rgb_image = CvBridge().compressed_imgmsg_to_cv2(msg)
            self.hsv_image = cv.cvtColor(self.rgb_image, cv.COLOR_BGR2HSV)
            self.create_mask()

    def create_mask(self):
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

        # # # Now a grey version of image
        # grayed_image = cv.cvtColor(masked_img, cv.COLOR_RGB2GRAY)  # convert image to grayscale
        # grayed_msg = CvBridge().cv2_to_imgmsg(grayed_image)
        # grayed_pub.publish(grayed_msg)

        # # Now lets compute the centroid
        # h, w = grayed_image.shape
        # search_top = int(3*h/4)
        # search_bot = int(search_top + 20)
        # grayed_image[0:search_top, 0:w] = 0
        # grayed_image[search_bot:h, 0:w] = 0
        # M = cv.moments(grayed_image)
        # if M['m00'] > 0:
        #     cx = int(M['m10']/M['m00'])
        #     cy = int(M['m01']/M['m00'])
        #     cv.circle(rgb_image, (cx, cy), 50, (0,0,255), -1)
        # centroid_msg = CvBridge().cv2_to_imgmsg(rgb_image)
        # centroid_pub.publish(centroid_msg)

        # # Now blurr
        # blurred_image = cv.GaussianBlur(grayed_image, (15, 15), 0)  # blur image with a 5x5 kernel
        # blurred_msg = CvBridge().cv2_to_imgmsg(blurred_image)
        # blurred_pub.publish(blurred_msg)

        # # Lets try countours
        # ret, thresh = cv.threshold(
        #     blurred_image, 127, 255, cv.THRESH_BINARY_INV
        # )  # create an threshold
        # contours, hierachy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        # contour_image = cv.drawContours(thresh, contours, -1, (255, 255, 255), 20)
        # contour_msg = CvBridge().cv2_to_imgmsg(contour_image)
        # contour_pub.publish(contour_msg)

    def dynamic_cb(self, config, level):
        rospy.loginfo("Dynamic Config callback {lb_h}:{lb_s}:{lb_v} {ub_h}:{ub_s}:{ub_v}".format(**config))
        self.config = config
        self.param_ready = True
        return config

    def __init__(self):
        self.param_ready = False
        self.cam_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.cv_callback)
        self.masked_pub = rospy.Publisher("/camera/masked/compressed", CompressedImage, queue_size=1)
        self.grayed_pub = rospy.Publisher("/camera/rgb/grayed", Image, queue_size=1)
        self.blurred_pub = rospy.Publisher("/camera/rgb/blurred", Image, queue_size=1)
        self.contour_pub = rospy.Publisher("/camera/rgb/contour", Image, queue_size=1)
        self.centroid_pub = rospy.Publisher("/camera/rgb/centroid", Image, queue_size=1)
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

