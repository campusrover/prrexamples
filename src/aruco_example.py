#!/usr/bin/env python3
import rospy

import tf2_ros
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import FiducialTransformArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class ArucoExample():

    def fiducial_cb(self, msg):
        if len(msg.transforms) != 0:
            self.detected_target = msg.transforms[0].fiducial_id
        else:
            self.detected_target = None
        print(f"Target in sight: {self.detected_target}")
        # if (self.state == "none_in_sight"):
        #     locate_fiducial()

    def locate_fiducial():
        # Rotate in place until a fiducial is in sight
        pass

    def __init__(self):
        rospy.init_node('aruco_sample')
        self.my_sub = rospy.Subscriber(
            '/fiducial_transforms', FiducialTransformArray, self.fiducial_cb)
        self.rate = rospy.Rate(10)
        self.state = "none_in_sight"

    def run(self):
        while not rospy.is_shutdown() and self.state != "done_looking":
            self.rate.sleep()


aruco_example = ArucoExample()
aruco_example.run()
