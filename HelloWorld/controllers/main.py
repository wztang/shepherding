#!/usr/bin/env python3
# This is the main control loop running in each argos robot

# /* Import Packages */
#######################################################################
import random, math
import time, sys, os

import json
experimentFolder = os.environ['EXPERIMENTFOLDER']
sys.path += [os.environ['MAINFOLDER'], \
             os.environ['EXPERIMENTFOLDER']+'/controllers', \
             os.environ['EXPERIMENTFOLDER']
            ]

from controllers.movement import RandomWalk, Navigate, Odometry, OdoCompass, GPS
from controllers.groundsensor import ResourceVirtualSensor, Resource, GroundSensor
from controllers.erandb import ERANDB
from controllers.rgbleds import RGBLEDs
from controllers.aux import *
from controllers.aux import Timer
from controllers.statemachine import *

from controllers.control_params import params as cp


# /* Global Variables */
#######################################################################
global robot

global startFlag
global notdonemod
notdonemod=False
startFlag = False

global txList, tripList, submodules
txList, tripList, submodules = [], [], []

global clocks, counters, logs, txs
clocks, counters, logs, txs = dict(), dict(), dict(), dict()

global estimate, totalWhite, totalBlack, byzantine, byzantine_style, log_folder
TPS = 5000

estimate =0
totalWhite =0
totalBlack=0

byzantine = 0
# /* Logging Levels for Console and File */
#######################################################################
import logging
loglevel = 10
logtofile = False 

# /* Experiment Global Variables */
#######################################################################

clocks['peering'] = Timer(5)
clocks['sensing'] = Timer(2)
clocks['ubi'] = Timer(100)
clocks['block']   = Timer(120)
clocks['newround'] = Timer(20)
clocks['voting'] = Timer(30)
global geth_peer_count



####################################################################################################################################################################################
#### INIT STEP #####################################################################################################################################################################
####################################################################################################################################################################################

def init():
    global clocks,counters, logs, submodules, me, rw, nav, odo, gps, rb, w3, fsm, rs, erb, rgb,odo2,gs, byzantine_style, log_folder
    robotID = str(int(robot.variables.get_id()[2:])+1)
    robotIP = '127.0.0.1'
    robot.variables.set_attribute("id", str(robotID))
    robot.variables.set_attribute("byzantine_style", str(0))
    robot.variables.set_attribute("consensus_reached",str("false"))
    robot.variables.set_attribute("scresources", "[]")
    robot.variables.set_attribute("foraging", "")
    robot.variables.set_attribute("state", "")
    robot.variables.set_attribute("quantity", "0")
    robot.variables.set_attribute("block", "")
    robot.variables.set_attribute("block", "0")
    robot.variables.set_attribute("hash", str(hash("genesis")))
    robot.variables.set_attribute("state_hash", str(hash("genesis")))

    # /* Initialize Console Logging*/
    #######################################################################
    log_folder = experimentFolder + '/logs/' + robotID + '/'

    # Monitor logs (recorded to file)
    name =  'monitor.log'
    os.makedirs(os.path.dirname(log_folder+name), exist_ok=True) 
    logging.basicConfig(filename=log_folder+name, filemode='w+', format='[{} %(levelname)s %(name)s] %(message)s'.format(robotID))
    logging.getLogger('sc').setLevel(10)
    logging.getLogger('w3').setLevel(10)
    logging.getLogger('poa').setLevel(10)
    robot.log = logging.getLogger()
    robot.log.setLevel(0)

    # /* Initialize submodules */
    #######################################################################
    # # /* Init web3.py */
    robot.log.info('Initialising Python Geth Console...')
    #w3 = Node(robotID, robotIP, 1233 + int(robotID), ProofOfAuthority(GENESIS))

    # /* Init an instance of peer for this Pi-Puck */
    me = Peer(robotID, robotIP)

    # /* Init E-RANDB __listening process and transmit function
    robot.log.info('Initialising RandB board...')
    erb = ERANDB(robot, cp['erbDist'] , cp['erbtFreq'])

    #/* Init Resource-Sensors */
    robot.log.info('Initialising resource sensor...')
    rs = ResourceVirtualSensor(robot)
    
    # /* Init Random-Walk, __walking process */
    robot.log.info('Initialising random-walk...')
    #rw = RandomWalk(robot, cp['scout_speed'])

    # /* Init Navigation, __navigate process */
    robot.log.info('Initialising navigation...')
    nav = Navigate(robot, cp['recruit_speed'])
    
    # /* Init odometry sensor */
    robot.log.info('Initialising Odo...')
    odo2 = Odometry(robot)
    
    # /* Init odometry sensor */
    robot.log.info('Initialising odometry...')
    odo = OdoCompass(robot)

    # /* Init GPS sensor */
    robot.log.info('Initialising gps...')
    gps = GPS(robot)

    # /* Init LEDs */
    rgb = RGBLEDs(robot)

    # /* Init Finite-State-Machine */
    fsm = FiniteStateMachine(robot, start = States.IDLE)

    # /* Init Ground sensor */
    robot.log.info('Initialising Ground sensor...')
    gs = GroundSensor(robot)
    
    # List of submodules --> iterate .start() to start all
    submodules = [erb,gs]

#########################################################################################################################
#### CONTROL STEP #######################################################################################################
#########################################################################################################################
global pos
pos = [0,0]
global last
last = 0
counter = 0
global checkt


def compute_total_force_and_movement(robot, robot_position, erb_peers):
    # Compute the total force from all peers
    erb_enodes = {peer.id for peer in erb.peers}
    # 将JSON字符串转换回字典
    inherent_properties = json.loads(robot.variables.get_attribute("inherent_properties"))
    robot_width = 0.07 # Assuming some constant value for robot width
    K_ij = float(robot.variables.get_attribute("K"))  # Assuming some constant value for K_ij
    alpha = float(robot.variables.get_attribute("alpha")) # Assuming some constant value for alpha
    beta = float(robot.variables.get_attribute("beta")) # Assuming some constant value for beta
    V_0 = float(robot.variables.get_attribute("V_0")) # cm/s
    L_i = float(robot.variables.get_attribute("L")) # m # Assuming some constant value for L_ij (equilibrium distance)
    R_i = L_i * float(robot.variables.get_attribute("R_rate"))
    orientation_angle_i =  odo.getOrientation()  # Assuming this returns the orientation angle in degrees
    motion_position_i = robot_position
    geometric_position_i = [motion_position_i[0] + R_i * math.cos(orientation_angle_i), motion_position_i[1] + R_i * math.sin(orientation_angle_i)]
    total_F_x = 0
    total_F_y = 0

    # # Control parameters for movement
    # if int(me.id) <= 0:
    #     robot.epuck_wheels.set_speed(0, 0)
    #     rgb.setLED(rgb.all, ['black', 'black', 'black'])
    # elif int(me.id) <= 0:
    #     rgb.setLED(rgb.all, ['blue', 'blue', 'blue'])
    #     V_0 = 0
    # else:
    #     rgb.setLED(rgb.all, ['red', 'red', 'red'])

    for peer in erb_peers:
        L_j = float(inherent_properties[f"{peer.id}"]["L"])
        R_j = L_j * float(inherent_properties[f"{peer.id}"]["R_rate"])
        orientation_angle_j = float(inherent_properties[f"{peer.id}"]["direction"])
        motion_position_j = [motion_position_i[0] + peer.range*math.cos(peer.bearing+orientation_angle_i), motion_position_i[1] + peer.range*math.sin(peer.bearing+orientation_angle_i)]
        geometric_position_j = [motion_position_j[0] + R_j*math.cos(orientation_angle_j), motion_position_j[1] + R_j*math.sin(orientation_angle_j)]
        L_ij = (L_i + L_j) / 2
        geometric_x = geometric_position_j[0]-geometric_position_i[0]
        geometric_y = geometric_position_j[1]-geometric_position_i[1]
        geometric_distance = (geometric_x**2 + geometric_y**2)**0.5
        geometric_direction = math.atan2(geometric_y,geometric_x)
        # if me.id == "2" and peer.id == 1:
        # print("")
        # print(f"id:{me.id},direction:{orientation_angle_i:.6f},motion_position:({motion_position_i[0]:.6f},{motion_position_i[1]:.6f}),geometric_position:({geometric_position_i[0]:.6f},{geometric_position_i[1]:.6f})")
        # print(f"N_id:{peer.id},direction:{orientation_angle_j:.6f},motion_position:({motion_position_j[0]:.6f},{motion_position_j[1]:.6f}),geometric_position:({geometric_position_j[0]:.6f},{geometric_position_j[1]:.6f})")     
        # print(f"L:{L_ij}")  
        # print(f"peer:distance:{peer.range:.6f},        bearing  :{peer.bearing+orientation_angle_i:.6f},(x,y):({peer.range*math.cos(peer.bearing+orientation_angle_i):.6f},{peer.range*math.sin(peer.bearing+orientation_angle_i):.6f})")  
        # print(f"cau :distance:{geometric_distance:.6f},direction:{geometric_direction:.6f},(x,y):({geometric_x:.6f},{geometric_y:.6f})")
        if geometric_distance < L_ij:
            # Force contribution from this peer
            F = K_ij / L_ij * (geometric_distance - L_ij)
            total_F_x +=  F * math.cos(geometric_direction)
            total_F_y += F * math.sin(geometric_direction)
            # if me.id == "2" and peer.id == 1:
            # print(f"F:{F},cos:{math.cos(geometric_direction)},sin:{math.sin(geometric_direction)}")
            
    # Compute the total force from the boundary
    boundary_distance = cp['arena']['radius'] - (geometric_position_i[0] ** 2 + geometric_position_i[1] ** 2) ** 0.5
    boundary_direction = math.atan2(geometric_position_i[1], geometric_position_i[0])
    if boundary_distance < L_i:
        F = K_ij / L_i * (boundary_distance - L_i)
        total_F_x +=  F * math.cos(boundary_direction)
        total_F_y += F * math.sin(boundary_direction)
    
    # Calculate movement based on the total force
    force_parallel = total_F_x * math.cos(orientation_angle_i) + total_F_y * math.sin(orientation_angle_i)
    force_perpendicular = -total_F_x * math.sin(orientation_angle_i) + total_F_y * math.cos(orientation_angle_i)
    
    v_t = (V_0 + alpha * force_parallel)
    w_t = (beta * force_perpendicular)
    
    # Calculate wheel speeds
    left_wheel_speed = (v_t - robot_width * w_t / 2) 
    right_wheel_speed = (v_t + robot_width * w_t / 2)
    # if me.id == "2":
    # print(f"id:{me.id},force:({total_F_x},{total_F_y})({force_parallel},{force_perpendicular}),v:{v_t},w_t:{w_t}")
    # print(f"{me.id},{orientation_angle_i}, ({robot_position[0]},{robot_position[1]}),({motion_position_i[0]},{motion_position_i[1]}),({geometric_position_i[0]},{geometric_position_i[1]})" )
    robot.epuck_wheels.set_speed(left_wheel_speed, right_wheel_speed)


def controlstep():
    global counter, last, pos, clocks, counters, startFlag, startTime, notdonemod, odo2, checkt, byzantine, byzantine_style, log_folder
    global estimate, totalWhite, totalBlack
        
    for clock in clocks.values():
        clock.time.step()
        checkt = clock.time.time_counter

    if not startFlag:
        startFlag = True
        startTime = 0
        robot.log.info('--//-- Starting Experiment --//--')
        robot.log.info(f"K alpha beta V_0 L R_rate")
        K = float(robot.variables.get_attribute("K"))
        alpha = float(robot.variables.get_attribute("alpha"))
        beta = float(robot.variables.get_attribute("beta"))
        V_0 = float(robot.variables.get_attribute("V_0"))
        L = float(robot.variables.get_attribute("L"))
        R_rate = float(robot.variables.get_attribute("R_rate"))
        robot.log.info(f"{K} {alpha} {beta} {V_0} {L} {R_rate}")
        robot.log.info(f"motion_x motion_y direction")
        for module in submodules:
            try:
                module.start()
            except:
                robot.log.critical('Error Starting Module: %s', module)
                sys.exit()
        for log in logs.values():
            log.start()
        for clock in clocks.values():
            clock.reset()
        byzantine_style = int(robot.variables.get_attribute("byzantine_style"))

    else:
        ###########################
        ######## ROUTINES #########
        ###########################

        def peering():
            global estimate, totalWhite, totalBlack, checkt, byzantine, byzantine_style, log_folder

            # Get the current peers from ERB
            erb_enodes = {peer.id for peer in erb.peers}

            # Turn on LEDs according to geth peer count
            # rgb.setLED(rgb.all, rgb.presets.get(len(erb_enodes), 3*['red']))

        # Perform submodules step
        for module in [erb, rs, gs, odo]:
            module.step()

        # Perform clock steps
        for clock in clocks.values():
            clock.time.step()

        if clocks['peering'].query():
            peering()


        # Get the current position of the robot
        robot_position = gps.getPosition()
        compute_total_force_and_movement(robot, robot_position, erb.peers)
        robot.log.info(f"{robot_position[0]} {robot_position[1]} {odo.getOrientation()}")

def reset():
    pass

def destroy():
        
    print('Killed robot '+ me.id)

#########################################################################################################################
#########################################################################################################################
#########################################################################################################################
