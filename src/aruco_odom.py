#!/usr/bin/env python  
import rospy

import math
import tf2_ros
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Create a node and allocate a tf buffer and a tf listener.

class ArucoOdom():
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.odom_pub = rospy.Publisher('/fiducial/twist', Twist, queue_size=1)

    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                self.trans = self.tfBuffer.lookup_transform("base_link", 'fiducial_2', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException) as ex:
                rate.sleep()
                rospy.logerr(str(ex))
                continue
            quat = self.trans.transform.rotation
            trans = self.trans.transform.translation
            msg = Twist()
            msg.linear.x = trans.x
            msg.linear.y = trans.y
            msg.linear.z = trans.z
            ( msg.angular.x, msg.angular.y, msg.angular.z) = euler_from_quaternion ([quat.x, quat.y, quat.z, quat.w])
            self.odom_pub.publish(msg)

# Main function.
if __name__ == "__main__":
    rospy.init_node('tf2_aruco_listener')
    try:
        ArucoOdom().run()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
