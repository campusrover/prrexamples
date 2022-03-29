#!/usr/bin/env python  
import rospy

import math
import tf2_ros
# import geometry_msgs.msg
import nav_msgs.msg

# Create a node and allocate a tf buffer and a tf listener.

class ArucoOdom():
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(tfBuffer)
        self.odom_pub = rospy.Publisher('/fiducial/odom', nav_msgs.msg.Odometry, queue_size=1)

    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                self.trans = tfBuffer.lookup_transform(base_link, 'fiducial1', rospy.Time())
            except:
                (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.         ExtrapolationException):
                    rate.sleep()
                    rospy.loginfo("tf trans error")
                    continue
                rospy.loginfo("Transform: ")
                msg = nav_msgs.msg.Odometry()
                msg.child_frame_id = "X"
                msg.pose.position.x = 0
                msg.pose.position.y = 0
                msg.pose.position.z = 0
                msg.pose.orientation.x = 0
                msg.pose.orientation.y = 0
                msg.pose.orientation.z = 0
                msg.pose.orientation.w = 0
                self.odom_pub.publish(msg)

# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node('tf2_aruco_listener')
    # Go to class functions that do all the heavy lifting.
    try:
        ArucoOdom().run()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()




# And we loop, 10x per second
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
# This is the most important line. Requests the transform between turtle1 and turtle_name
            trans = tfBuffer.lookup_transform(base_link, 'fiducial1', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.         ExtrapolationException):
            rate.sleep()
            continue
        rospy.loginfo("Transform: ")
        msg = nav_msgs.msg.Odometry()
        msg.

        msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

# And publish it to drive turtle2
        turtle_vel.publish(msg)

        rate.sleep()
