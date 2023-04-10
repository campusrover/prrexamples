#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class CheckCmdVel:
    def __init__(self):
        rospy.init_node('check_cmdvel')
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10)

    def run(self):
        self.counter = 0
        self.state = "running"

        while not rospy.is_shutdown() and self.state != "done":
            self.counter += 1
            if 0 < self.counter < 25:
                print("1")
                self.twist = Twist()
                self.twist.linear.x = 0.3
            elif 25 < self.counter < 50:
                print("2")
                self.twist = Twist()
                self.twist.linear.x = -0.3
            elif 50 < self.counter < 75:
                print("3")
                self.twist = Twist()
                self.twist.linear.x = 0.3
            elif 75 < self.counter < 100:
                print("4")
                self.twist = Twist()
                self.twist.linear.x = 0.05
                self.twist.angular.z = 0.5
            elif 100 < self.counter < 125:
                print("5")
                self.twist = Twist()
                self.twist.linear.x = 0.05
                self.twist.angular.z = -0.5
            elif 125 < self.counter < 300:
                print("6")
                self.twist = Twist()
                self.twist.angular.z = -0.5
                self.twist.linear.x = 0.1
            elif self.counter > 200:
                self.state = "done"
            self.twist_pub.publish(self.twist)
            self.rate.sleep()

c = CheckCmdVel()
c.run()

