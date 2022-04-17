#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from prrexamples.msg import Robogym 



class RoboGym:
    def __init__(self):
        self.speed_trans = 0
        self.speed_ang = 0
        self.pub_rate = 0          # How many pubs happen per second
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.sub_cli = rospy.Subscriber('cli', Robogym, self.cli_callback)

    def cli_callback(self, msg):
        pass

    def run(self):
        pass

# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("robogym")
    try:
        RoboGym()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()


