#!/usr/bin/env python3
import rospy

import tf2_ros
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import FiducialTransformArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class ArucoExample():

    def fiducial_cb(self, msg):
        #rospy.loginfo(f"Targets in sight: {len(msg.transforms)}")
        if len(msg.transforms) != 0:
            self.detected_target = msg.transforms[0].fiducial_id
            rot_q = msg.transforms[0].transform.rotation
            rot_e = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
            tra = msg.transforms[0].transform.translation
#           print(f'{tra.x:3.2} {tra.y:3.2} {tra.z:3.2} {rot_e[0]:3.2} {rot_e[1]:3.2} {rot_e[2]:3.2}')

        else:
            self.detected_target = None

    def __init__(self):
        rospy.init_node('aruco_sample')
        self.fid_sub = rospy.Subscriber(
            '/fiducial_transforms', FiducialTransformArray, self.fiducial_cb)
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10)
        self.state = "none_in_sight"
        self.visited = []

    def run(self):
        rospy.loginfo("Running")
        while not rospy.is_shutdown() and self.state != "done_looking":
            self.rate.sleep()

aruco_example = ArucoExample()
aruco_example.run()
