#!/usr/bin/env python3

import os
import sys
import random
import logging, time, json
import libpy_loop_function_interface
from hexbytes import HexBytes

from controllers.aux import Vector2D, Logger, Timer, Accumulator, mydict, identifiersExtract, Counter
from controllers.groundsensor import Resource

from controllers.control_params import params as cp
#from loop_params import params as lp
#from loop_helpers import *

import math

# Initialize loop function interface
loop_function_interface = libpy_loop_function_interface.CPyLoopFunction()

# Global variables
global startFlag, stopFlag, startTime
startFlag = False
stopFlag = False
startTime = 0

# Initialize timers/accumulators/logs
global clocks, accums, logs, other,addspacebetweenrobots
addspacebetweenrobots = 0
clocks, accums, logs, other = dict(), dict(), dict(), dict()
other['countsim'] = Counter()
# Function definitions
def init():
    global addspacebetweenrobots
    # radius = 0.8
    # cirle_length = 2*math.pi * radius
    # wall_robot_num = math.floor(cirle_length / 0.2)
    # print("Wall robot num: ", wall_robot_num)
    # angle = (2 * math.pi) / wall_robot_num
    # for robot in allrobots:
    #     robot.id = int(robot.variables.get_attribute("id"))
    #     if robot.id <= wall_robot_num:
    #         loop_function_interface.AddRobotArena(radius * math.cos(angle*(robot.id - 1)), radius * math.sin(angle*(robot.id - 1)), robot.id - 1)
            
            
        

    #"""
    # byzantines = random.sample(allrobots, k=int(lp['environ']['NUMBYZANTINE']))
    # for robot in byzantines:
    #    robot.variables.set_attribute("byzantine_style", lp['environ']['BYZANTINESWARMSTYLE'])
    #    print("Making robot", robot.variables.get_attribute("id"), "Byzantine.")
    #    robot.variables.set_attribute("isByz","True")
    #"""

def pre_step():
    global startFlag, startTime
    if not startFlag:
        startTime = 0
    robotsDirection = dict()
    for robot in allrobots:
        robot.id = robot.variables.get_attribute("id")
        robotsDirection[robot.id] = robot.position.get_orientation()
    # 将字典转换为JSON字符串
    robotsDirection_str = json.dumps(robotsDirection)
    for robot in allrobots:
        robot.variables.set_attribute("neighbor_direction", robotsDirection_str)
    """
    if other['countsim'].count == 5:
        try:
            loop_function_interface.AddRobotArena(-0.5, 0.0, 0)
            loop_function_interface.AddRobotArena(0.5, 0.0, 1)
            loop_function_interface.AddRobotArena(-0.5, 0.0, 2)
            loop_function_interface.AddRobotArena(0.6, 0.93, 3)


        except Exception as e:
            print(f"An error occurred: {e}")
    """
def post_step():
    global startFlag, clocks, accums
    other['countsim'].step()
    """
    if other['countsim'].count == 150:
        try:
            loop_function_interface.AddRobotArena(0.5, 0.0, 0)
            loop_function_interface.AddRobotArena(0.5, 0.1, 1)
            loop_function_interface.AddRobotArena(0.5, 0.2, 2)
            loop_function_interface.AddRobotArena(0.5, 0.3, 3)

        except Exception as e:
            print(f"An error occurred: {e}")
    """

def is_experiment_finished():
    pass

def reset():
    pass

def destroy():
    pass

def post_experiment():
    print("Finished from Python!")