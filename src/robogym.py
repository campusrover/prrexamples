#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from prrexamples.msg import Robogym 



class RoboGym:
    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.sub_cli = rospy.Subscriber('cli', Robogym, self.cli_callback)
        self.reset()

    def reset(self):
        self.speed_trans = 0
        self.mode = "standard"
        self.countdown = 0
        self.speed_ang = 0
        self.speed_linear = 0
        self.pub_rate = 0.5
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.sub_cli = rospy.Subscriber('cli', Robogym, self.cli_callback)
        self.last_twist = Twist()


# Modes are:
# standard - publishes cmd_vel 'rate' times per second
# idle - doesn't publish cmd_vel
# limited - goes for a certain count, time or distance
#
# params are:
# rate = hz for cmd_vel pubs
# lin = linear speed
# ang = angular speed
# couunt = counter 

    def cli_callback(self, msg):
        if (msg.command == "rate"):
            self.pub_rate = msg.arg1
            if self.pub_rate == 0:
                self.mode = "idle"
        elif (msg.command == "lin"):
            self.speed_linear = msg.arg1
        elif msg.command == "rot":
            self.speed_ang = msg.arg1
        elif msg.command == "count":
            self.countdown = msg.arg1
        elif msg.command == "reset":
            self.reset()
        elif msg.command == "standard":
            self.mode = "standard"
            self.pub_rate = 0.5
        elif msg.command == "idle":
            self.mode = "idle"
        elif msg.command == "limit":
            self.mode = "limit"


    def step(self):
        print("publish cmd_vel")
        self.last_twist = Twist()
        self.last_twist.linear.x = self.speed_linear
        self.last_twist.angular.z = self.speed_ang
        self.pub_cmd_vel.publish(self.last_twist)
        if self.pub_rate == 0:
            print("step with pub rate 0 is a bug")
        else:
            rospy.sleep(1.0/self.pub_rate)

    def run(self):
        while not rospy.is_shutdown():
            if self.mode == "limit":
                print("counting down")
                self.countdown -= 1
                if self.countdown == 0:
                    self.mode = "idle"
                    self.pub_rate = 0.0
                else:
                    self.step()
            elif self.mode == "idle":
                print("idle")
                rospy.sleep(1)
            elif self.mode == "standard":
                self.step()

# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("robogym")
    try:
        RoboGym().run()
    except rospy.ROSInterruptException:
        pass
    