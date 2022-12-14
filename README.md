# ECE 470 Final Project
## ECE 470 Project update 1

## Table of contents
* [Introduction](#introduction)
* [Technologies](#technologies)
* [Task Objective](#task-objective)
* [Current Codebase](#current-codebase)
* [File Descriptions](#file-descriptions)
  * [lab2_exec.py](#lab2_exec.py)
  * [lab2_spawn.py](#lab2_spawn.py)


## Introduction
The objective of this project is to use the Universal Robots arm to complete a game of cup pong. The robotic arm will pick up the ball from a designated location using the suction gripper, and attempt to throw the ball into a cup places at another designated location. The code base includes the feedback from the suction gripper indicating the presence of the ball.
This project was done for the course project of ECE 470 at UIUC for the fall 2022 semester
	
## Technologies
This project was created with Python 3
	
## Task Objective
The robotic arm will pick up the ball from a designated location using the gripper attachment. On the opposite side of the robot’s location will be the cup pong set up. There will be 6 cups organized into a pyramid location. The objective is for the robotic arm to grab the ball, then throw the ball against the table to bounce into its opponent’s cups. Each round, the robotic arm has 1 attempt to bounce the ball into the opponent’s cup. After the robot’s throw, a human member team will make an attempt to bounce a ball into the robot’s cups. The goal is for the robotic arm to bounce the ball into all of their opponent’s cups before the human team can do so.

## Current Codebase
The current code shows that we can interface with the simulator, move the robot, and  access some sensor measurements. We have tested the code to move the robot around and create a ball to be used as a ping pong ball. We will then be aiming to attempt to throw the ball at some velocity in the direction of the cup locations.
We have tested the sensors related to the gripper to identify when a ball is being picked up or not(suction gripper). This had been done by subscribing to the gripper input, and then implementing a callback function to get the state of the gripper(holding a ball vs not holding a ball).

## File Descriptions
The compressed folder of all relevant files is included on this GitHub repository in the releases section. Some important files for the project are listed below.
### lab2_exec.py
Main Python file for simulating robot. Files are adapted from Lab 2 of the ECE 470 course, and will be further edited to implement the solution for this final project. 

In this file, the possible positions of the end effector of the robotic arm are listed. Then a subscriber is called to receive sensor information from the suction gripper. Additionally, the move_arm and move_block functions describe the ways and points the robot arm shall move to in order to pick up objects.
### lab2_spawn.py
Python file for creating objects in Gazebo.

