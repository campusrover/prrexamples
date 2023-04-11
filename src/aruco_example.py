#!/usr/bin/env python3
import rospy

import tf2_ros
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import FiducialTransformArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Vector3


class ArucoExample():

    def fiducial_cb(self, msg):
        self.adopt_target(msg)
        if self.located != None:
            self.none_count = 0
            tra, rot_e = self.calc_transform()
            print(f'{tra.x:3.2} {tra.y:3.2} {tra.z:3.2} {rot_e[0]:3.2} {rot_e[1]:3.2} {rot_e[2]:3.2}')
            self.debug.x = tra.z
            self.debug.z = rot_e[1]

            if tra.z > 0.5:
                print("going")
                if rot_e[1] > 0.02:
                    print("veer to right")
                    self.set_twist(0.05, -0.1)
                elif rot_e[1] < -0.02:
                    self.set_twist(0.05, 0.1)
                    print("veer to left")
                else:
                    self.set_twist(0.2, 0)
                    print("straight")
            else:
                print("done_looking")
                self.set_twist(0, 0)
                self.state = "done_looking"
        else:
            print("None")
            self.none_count += 1
            if self.none_count > 5:
                self.set_twist(0, 0)
                print("lost")                
                self.state = "done_looking"


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
    
    def set_twist(self, x, z):
        self.twist.linear.x = x
        self.twist.angular.z = z

    def __init__(self):
        rospy.init_node('aruco_sample')
        self.fid_sub = rospy.Subscriber(
            '/fiducial_transforms', FiducialTransformArray, self.fiducial_cb)
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.debug_pub = rospy.Publisher('/debug', Vector3, queue_size=1)
        self.rate = rospy.Rate(10)
        self.state = "none_in_sight"
        self.visited = []
        self.twist = Twist()
        self.debug = Vector3()
        self.detected_target = None
        self.located = None
        self.none_count = 0


    def run(self):
        rospy.loginfo("Running")
        self.state = "lookings"
        while not rospy.is_shutdown() and self.state != "done_looking":
            self.twist_pub.publish(self.twist)
            # self.debug_pub.publish(self.debug)
            self.rate.sleep()

aruco_example = ArucoExample()
aruco_example.run()
