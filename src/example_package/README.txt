### How to run

* please use `roslaunch example_package example_package.launch`
* You must mark all three nodes as executable using the " $ chmod +x <file_name>.py " command.
* This roslaunch command will launch the three nodes and show the robot simply driving straight.

### Purpose

The purpose of this package is to show how many nodes might link together in a complex ROS program.

There is an example of lidar processing in the scan_values_handler.py file although it currently 
does not do anything at the moment. This is where you could implement your own lidar data processing  as you see fit. There is also a pid.py file which is a skeleton of a PID controller node to help smoothly control the robot. PID controllers are very useful and you should read a bit about them before filling in the PID code. The driver.py node is the main control loop of our program which makes higher level decisions about which control instructions to follow.

This package is primarily used to illustrate good ROS coding practices and you should use these 
ideas in your assignments to come.
