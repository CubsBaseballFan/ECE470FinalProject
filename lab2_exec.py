#!/usr/bin/env python

'''
Project Update 1 Code To Demonstrate Movement of Robot and Sensor Measurements in Gazebo
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([155.15, -71.18, 84.9, -103.54, -91.31, -24.63])
Q1 = np.radians([143.6, -66.6, 78.91, -102.39, -91.31, -36])
Q2 = np.radians([154.08, -71.05, 83.01, -101.82, -91.29, -25.51])
Q3 = np.radians([166, -72.35, 84.3, -101.54, -91.21, -13.57])

# Different positions to check for objects
Q11 = [143.62*pi/180.0, -54.64*pi/180.0, 94.37*pi/180.0, -129.81*pi/180.0, -91.33*pi/180.0, -36.08*pi/180.0]
Q12 = [143.63*pi/180.0, -48.99*pi/180.0, 96.11*pi/180.0, -137.20*pi/180.0, -91.32*pi/180.0, -36.10*pi/180.0]
Q13 = [143.77*pi/180.0, -43.29*pi/180.0, 95.98*pi/180.0, -142.74*pi/180.0, -91.31*pi/180.0, -35.96*pi/180.0]

Q21 = [154.10*pi/180.0, -57.55*pi/180.0, 100.18*pi/180.0, -132.49*pi/180.0, -91.31*pi/180.0, -25.6*pi/180.0]
Q22 = [154.11*pi/180.0, -52.60*pi/180.0, 101.71*pi/180.0, -138.97*pi/180.0, -91.30*pi/180.0, -25.61*pi/180.0]
Q23 = [153.88*pi/180.0, -46.92*pi/180.0, 102.96*pi/180.0, -145.89*pi/180.0, -91.30*pi/180.0, -25.86*pi/180.0]

Q31 = [165.94*pi/180.0, -58.44*pi/180.0, 101.94*pi/180.0, -133.12*pi/180.0, -91.25*pi/180.0, -13.75*pi/180.0]
Q32 = [165.94*pi/180.0, -52.61*pi/180.0, 103.10*pi/180.0, -140.11*pi/180.0, -91.24*pi/180.0, -13.78*pi/180.0]
Q33 = [166.03*pi/180.0, -47.35*pi/180.0, 103.56*pi/180.0, -145.81*pi/180.0, -91.23*pi/180.0, -13.71*pi/180.0]

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]
############### Your Code End Here ###############
############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""

############### Your Code End Here ###############
def gripper_callback(msg):

    global analog_in_0
    
    analog_in_0 = msg.AIN0
"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q

    ### Hint: Use the Q array to map out your towers by location and "height".
    error = 0

    if start_loc == 0:
        move_arm(pub_cmd, loop_rate, Q1, 4.0, 4.0)
    elif start_loc == 1:
        move_arm(pub_cmd, loop_rate, Q2, 4.0, 4.0)
    elif start_loc == 2:
        move_arm(pub_cmd, loop_rate, Q3, 4.0, 4.0)
    else:
        print('not working')

    move_arm(pub_cmd, loop_rate, Q[start_loc][start_height], 4.0, 4.0)
    gripper(pub_cmd, loop_rate, suction_on)
    print(analog_in_0)
    print(current_io_0)
    #if digital_in_0 == 0:
        #error = error + 1
    # Delay to make sure suction cup has grasped the block
    time.sleep(1.0)

    if analog_in_0 < 2:
        print('Error: No ball found')
        gripper(pub_cmd, loop_rate, suction_off)
	print(analog_in_0)
	print(current_io_0)
        exit()

    rospy.loginfo("Sending goal 2 ...")
    if start_loc == 0:
        move_arm(pub_cmd, loop_rate, Q1, 4.0, 4.0)
    elif start_loc == 1:
        move_arm(pub_cmd, loop_rate, Q2, 4.0, 4.0)
    elif start_loc == 2:
        move_arm(pub_cmd, loop_rate, Q3, 4.0, 4.0)

    if end_loc == 0:
        move_arm(pub_cmd, loop_rate, Q1, 4.0, 4.0)
    elif end_loc == 1:
        move_arm(pub_cmd, loop_rate, Q2, 4.0, 4.0)
    elif end_loc == 2:
        move_arm(pub_cmd, loop_rate, Q3, 4.0, 4.0)

    rospy.loginfo("Sending goal 3 ...")
    move_arm(pub_cmd, loop_rate, Q[end_loc][end_height], 4.0, 4.0)


    gripper(pub_cmd, loop_rate, suction_off)
    print(analog_in_0)
    print(current_io_0)
    if end_loc == 0:
        move_arm(pub_cmd, loop_rate, Q1, 4.0, 4.0)
    elif end_loc == 1:
        move_arm(pub_cmd, loop_rate, Q2, 4.0, 4.0)
    elif end_loc == 2:
        move_arm(pub_cmd, loop_rate, Q3, 4.0, 4.0)

    return error


############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function
    sub_gripper = rospy.Subscriber('ur3/gripper_input',gripper_input, gripper_callback)

    ############### Your Code End Here ###############
    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    test = 0
    
    while(test == 0):
        start_pos = int(input("Enter start position <Either 0,1, or 2> "))
        end_pos = int(input("Enter end position <Either 0,1, or 2> "))
        if start_pos == end_pos:
            test = 0
        elif (start_pos != 0 | start_pos != 1 | start_pos != 2 ):
            test = 0
        elif (end_pos != 0 | end_pos != 1 | end_pos != 2 ):
            test = 0
        else:
            test = 1
   
        if start_pos == 0:
            if end_pos == 1:
                mid_pos = 2
            elif end_pos == 2:
                mid_pos = 1
            else:
                print('error')
        elif start_pos == 1:
            if end_pos == 2:
                mid_pos = 0
            elif end_pos == 0:
                mid_pos = 2
            else:
                print('error')
        if start_pos == 2:
            if end_pos == 1:
                mid_pos = 0
            elif end_pos == 0:
                mid_pos = 1
            else:
                print('error')

    print(start_pos , end_pos, mid_pos,  test)

    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

    move_arm(pub_command, loop_rate, home, 4.0, 4.0)
    move_block(pub_command, loop_rate, start_pos, 0, end_pos, 2)
    move_block(pub_command, loop_rate, start_pos, 1, mid_pos, 2)
    move_block(pub_command, loop_rate, end_pos, 2, mid_pos, 1)
    move_block(pub_command, loop_rate, start_pos, 2, end_pos, 2)
    move_block(pub_command, loop_rate, mid_pos, 1, start_pos, 2)
    move_block(pub_command, loop_rate, mid_pos, 2, end_pos, 1)
    move_block(pub_command, loop_rate, start_pos, 2, end_pos, 0)
    move_arm(pub_command, loop_rate, home, 4.0, 4.0)

    ############### Your Code End Here ###############

if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
