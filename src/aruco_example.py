#!/usr/bin/env python3
import rospy

import tf2_ros
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import FiducialTransformArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class ArucoExample():

    def fiducial_cb(self, msg):
        self.adopt_target(msg)
        if self.located != None:
            tra, rot_e = self.calc_transform()
            print(f'{tra.x:3.2} {tra.y:3.2} {tra.z:3.2} {rot_e[0]:3.2} {rot_e[1]:3.2} {rot_e[2]:3.2}')
        else;
            print("None")

    def __init__(self):
        rospy.init_node('aruco_sample')
        self.fid_sub = rospy.Subscriber(
            '/fiducial_transforms', FiducialTransformArray, self.fiducial_cb)
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10)
        self.state = "none_in_sight"
        self.visited = []

    def locate_target(self, msg, target):
        if len(msg.transforms) == 0:
            return None
        for i in range(len(msg.transforms)):
            if msg.transforms[i].fiducial_id == target:
                return msg.transforms[i]      
        return None

    def adopt_target(self, msg):
        if self.detected_target == None:
            if len(msg.transforms) != 0:
                self.located = msg.transforms[0]
        else:
            self.located = self.locate_target(msg, self.detected_target)
        if self.located != None:
            self.detected_target = self.located.fiducial_id

    def calc_transform(self):
        if self.located == None:
            return None
        rot_q = self.located.transform.rotation
        rot_e = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        tra = self.located.transform.translation
        return tra, rot_e

    def run(self):
        rospy.loginfo("Running")
        while not rospy.is_shutdown() and self.state != "done_looking":
            self.twist_pub.publish(self.twist)
            self.rate.sleep()

aruco_example = ArucoExample()
aruco_example.run()
