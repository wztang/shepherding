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
    for robot in allrobots:
        robot.id = robot.variables.get_attribute("id")
        robot.variables.set_attribute("K", "5")
        robot.variables.set_attribute("alpha", "5")
        robot.variables.set_attribute("beta", "150")
        if int(robot.id) <= 0:
            robot.variables.set_attribute("V_0", "0")
            robot.variables.set_attribute("L", "0.1")
            robot.variables.set_attribute("R_rate", "0.0")
        else:
            robot.variables.set_attribute("V_0", "3")
            robot.variables.set_attribute("L", "0.2")
            robot.variables.set_attribute("R_rate", "0.03")

def pre_step():
    global startFlag, startTime
    if not startFlag:
        startTime = 0
    inherent_properties = dict()
    for robot in allrobots:
        robot.id = robot.variables.get_attribute("id")
        inherent_properties[robot.id] = robot.position.get_orientation()
        inherent_properties[robot.id] = {"L": robot.variables.get_attribute("L"), "R_rate": robot.variables.get_attribute("R_rate"), "V_0": robot.variables.get_attribute("V_0"),"direction": robot.position.get_orientation()}
    # 将字典转换为JSON字符串
    inherent_properties_str = json.dumps(inherent_properties)
    for robot in allrobots:
        robot.variables.set_attribute("inherent_properties", inherent_properties_str)
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