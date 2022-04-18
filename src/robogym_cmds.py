#!/usr/bin/env python

# termios library allows access to terminal (shell) input
import sys, select, tty, termios
import rospy
from std_msgs.msg import String
from prrexamples.msg import Robogym 


class RobogymCmds:
    def __init__(self):
        self.cli_pub = rospy.Publisher("cli", Robogym, queue_size=1)

    def process_command(self, text_line):
        command = text_line.split()
        message = Robogym()
        if text_line == "" or command[0] == "help":
            print("Commands are: move, time, count, distance, reset")
        elif command[0] not in {'move', 'time', 'count', 'distance', 'reset'}:
            print("I don't know that command")
            print("Commands are: move, time, count, distance, reset")
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

            self.cli_pub.publish(message)

    def run(self):
        rate = rospy.Rate(5)
        #old_attr = termios.tcgetattr(sys.stdin)
        try:
            #tty.setcbreak(sys.stdin.fileno())
            command = ""
            print("\n> ", end="", flush=True)
            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
                    command = input("> ")
                    self.process_command(command)

                    # char = sys.stdin.read(1)
                    # if char == "\n":
                    #     self.process_command(command)
                    #     print("\n> ", end="", flush=True)
                    #     command = ""
                    # elif ord(char) == 127:
                    #     command = command[0:-1]
                    #     print("\r" + "> " + command, end="")
                    # else:
                    #     print(char, end="", flush=True)
                    #     command = command + char
                rate.sleep()
        finally:
            pass
        #     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)


if __name__ == "__main__":
    rospy.init_node("cli_publisher")
    r = RobogymCmds()
    r.run()
