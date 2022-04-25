#!/usr/bin/env python3

# termios library allows access to terminal (shell) input
import sys, select, tty, termios
import rospy
from std_msgs.msg import String
from prrexamples.msg import Robogym 


class RobogymCmds:
    def __init__(self):
        self.cli_pub = rospy.Publisher("cli", Robogym, queue_size=1)

    def do_cmd(self, command):
        print(command)
        message = Robogym()
        if command[0] == "help":
            print("Commands are: move, time, count, distance, reset")
            return False
        elif command[0] not in {'move', 'time', 'count', 'distance', 'reset', 'exit'}:
            print("I don't know that command")
            print("Commands are: move, time, count, distance, reset")
            return False
        elif command[0] == "exit":
            return True
        else:
            message.command = command[0]
            if (len(command) >= 2):
                message.lin = float(command[1])
            if (len(command) >= 3):
                message.ang = float(command[2])
            if (len(command) >= 4):
                message.rate = float(command[3])
            if (len(command) >= 5):
                message.lim = float(command[4])
            print(f"pubbing {message}")
            self.cli_pub.publish(message)
            return False
    
    def command_loop(self):
        exitnow = False
        if len(sys.argv) == 1:
            while not rospy.is_shutdown() and not exitnow:
                command = input(">>> ")
                command = command.split()
                exitnow = self.do_cmd(command)
    
    def command_once(self):
        try:
            rate = rospy.Rate(10)  # 10hz
            while not rospy.is_shutdown():
                connections = self.cli_pub.get_num_connections()
                print(f"Connections: {connections}")
                if connections > 0:
                    self.do_cmd(sys.argv[1:])
                    break
                rate.sleep()
        except rospy.ROSInterruptException as e:
            raise e
        
if __name__ == "__main__":
    rospy.init_node("rg")
    r = RobogymCmds()
    if len(sys.argv) == 1:
        r.command_loop()
    else:
        r.command_once()
