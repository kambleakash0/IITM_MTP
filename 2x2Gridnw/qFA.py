#!/usr/bin/python

from __future__ import absolute_import
from __future__ import print_function

import sumoPath
import sys
import optparse
# import subprocess
import random
import numpy as np
from matplotlib import interactive
# from matplotlib.ticker import FuncFormatter
import matplotlib.pyplot as plt
import traci
from sumolib import checkBinary

random.seed(42)  # make tests reproducible


def getPhase(val):
    if val == 0 or val == 1:
        return 0
    else:
        return 1


def getState(val):
    if val >= 0.60:
        return 2
    elif val >= 0.30:
        return 1
    else:
        return 0


def getStateIndex(state):
    idx = getState(state[0])*27 + getState(state[1])*9 + getState(state[2])*3 + getState(state[3])
    return idx


def getStateReturn(state):
    reward = 0
    for i in range(4):
        reward = reward + state[i]
    return reward*(-1)


def getSwitchTime(tlsPhase, currTLS, tNow, tlsID):
    if tlsPhase == 0 or tlsPhase == 2:
        elapsedSinceLastSwitch = tNow - (traci.trafficlight.getNextSwitch(tlsID) - traci.trafficlight.getPhaseDuration(tlsID))
        total = traci.trafficlight.getPhaseDuration(tlsID) + currTLS[0].phases[tlsPhase+1].duration
    else:
        elapsedSinceLastSwitch = tNow - (traci.trafficlight.getNextSwitch(tlsID) - traci.trafficlight.getPhaseDuration(tlsID) - currTLS[0].phases[tlsPhase-1].duration)
        total = traci.trafficlight.getPhaseDuration(tlsID) + currTLS[0].phases[tlsPhase-1].duration
    return (elapsedSinceLastSwitch / total)


def getQdash(phiSA, Sprime, theta):
    Actn = Sprime[3]
    lenFeature = Sprime[0]
    timeFeature = Sprime[1]
    phiDashSA = phiSA
    phiDashSA[Actn][Actn*2] = lenFeature
    phiDashSA[Actn][Actn*2 + 1] = timeFeature
    QdashSA = np.matmul(phiDashSA, theta)
    return QdashSA


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--gui", action="store_true", default=False, help="run the GUI version of sumo")
    options, args = optParser.parse_args()
    return options


def runQFS():
    # Initializations for the signal at 6
    stateVect_6 = [[]]
    policyPi_6 = []
    for i in range(0, 80, 1):
        policyPi_6.insert(i, random.randint(0, 5))
    QSA_6 = np.random.rand(81, 6)
    currTLS_6 = []
    currAction_6 = 0
    reward_6 = 0
    occupancy_6 = [0]*4

    # Initializations for the signal at 7
    stateVect_7 = [[]]
    policyPi_7 = []
    for i in range(0, 80, 1):
        policyPi_7.insert(i, random.randint(0, 5))
    QSA_7 = np.random.rand(81, 6)
    currTLS_7 = []
    currAction_7 = 0
    reward_7 = 0
    occupancy_7 = [0]*4

    # Initializations for the signal at 12
    stateVect_12 = [[]]
    policyPi_12 = []
    for i in range(0, 80, 1):
        policyPi_12.insert(i, random.randint(0, 5))
    QSA_12 = np.random.rand(81, 6)
    currTLS_12 = []
    currAction_12 = 0
    reward_12 = 0
    occupancy_12 = [0]*4

    # Initializations for the signal at 13
    stateVect_13 = [[]]
    policyPi_13 = []
    for i in range(0, 80, 1):
        policyPi_13.insert(i, random.randint(0, 5))
    QSA_13 = np.random.rand(81, 6)
    currTLS_13 = []
    currAction_13 = 0
    reward_13 = 0
    occupancy_13 = [0]*4

    # Initialization for the experiment
    Actions = [20, 25, 30, 20, 25, 30]
    avgTime = []
    numVehs = []
    EPSILON = 0.20
    ALPHA = 0.075
    GAMMA = 0.9
    step = 0
    totWaittimeSoFar = 0
    curWaittime = 0
    totVehsSoFar = 0
    currVehsHere = 0
    # Action!
    lanes_6 = traci.trafficlight.getControlledLanes("6")
    lanes_7 = traci.trafficlight.getControlledLanes("7")
    lanes_12 = traci.trafficlight.getControlledLanes("12")
    lanes_13 = traci.trafficlight.getControlledLanes("13")

    lanes = lanes_6 + lanes_7 + lanes_12 + lanes_13

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        # # # For the signal at 6 # # #
        for i in range(4):
            occupancy_6[i] = traci.lane.getLastStepOccupancy(lanes_6[2*i])
        tlsPhase_6 = traci.trafficlight.getPhase("6")
        currTLS_6 = traci.trafficlight.getCompleteRedYellowGreenDefinition("6")
        # Determine the state
        stateVect_6.insert(step, [occupancy_6, tlsPhase_6, 0, 0, 0])
        index_6 = getStateIndex(occupancy_6)
        reward_6 = getStateReturn(occupancy_6)
        # Take Action
        if random.uniform(0, 1) < EPSILON:
            currAction_6 = random.randint(0, 5)
        else:
            currAction_6 = policyPi_6[index_6]
        # Storage
        stateVect_6[step][2] = index_6
        stateVect_6[step][3] = currAction_6
        stateVect_6[step][4] = reward_6
        # Control Simulation
        if currAction_6 > 2:
            phaseToChange = 2
        else:
            phaseToChange = 0
        currTLS_6[0].phases[phaseToChange].duration = Actions[currAction_6]
        currTLS_6[0].phases[phaseToChange].minDur = Actions[currAction_6]
        currTLS_6[0].phases[phaseToChange].maxDur = Actions[currAction_6]
        traci.trafficlight.setCompleteRedYellowGreenDefinition("6", currTLS_6[0])
        # # # For the signal at 7 # # #
        for i in range(4):
            occupancy_7[i] = traci.lane.getLastStepOccupancy(lanes_7[2*i])
        tlsPhase_7 = traci.trafficlight.getPhase("7")
        currTLS_7 = traci.trafficlight.getCompleteRedYellowGreenDefinition("7")
        # Determine the state
        stateVect_7.insert(step, [occupancy_7, tlsPhase_7, 0, 0, 0])
        index_7 = getStateIndex(occupancy_7)
        reward_7 = getStateReturn(occupancy_7)
        # Take Action
        if random.uniform(0, 1) < EPSILON:
            currAction_7 = random.randint(0, 5)
        else:
            currAction_7 = policyPi_7[index_7]
        # Storage
        stateVect_7[step][2] = index_7
        stateVect_7[step][3] = currAction_7
        stateVect_7[step][4] = reward_7
        # Control Simulation
        if currAction_7 > 2:
            phaseToChange = 2
        else:
            phaseToChange = 0
        currTLS_7[0].phases[phaseToChange].duration = Actions[currAction_7]
        currTLS_7[0].phases[phaseToChange].minDur = Actions[currAction_7]
        currTLS_7[0].phases[phaseToChange].maxDur = Actions[currAction_7]
        traci.trafficlight.setCompleteRedYellowGreenDefinition("7", currTLS_7[0])
        # # # For the signal at 12 # # #
        for i in range(4):
            occupancy_12[i] = traci.lane.getLastStepOccupancy(lanes_12[2*i])
        tlsPhase_12 = traci.trafficlight.getPhase("12")
        currTLS_12 = traci.trafficlight.getCompleteRedYellowGreenDefinition("12")
        # Determine the state
        stateVect_12.insert(step, [occupancy_12, tlsPhase_12, 0, 0, 0])
        index_12 = getStateIndex(occupancy_12)
        reward_12 = getStateReturn(occupancy_12)
        # Take Action
        if random.uniform(0, 1) < EPSILON:
            currAction_12 = random.randint(0, 5)
        else:
            currAction_12 = policyPi_12[index_12]
        # Storage
        stateVect_12[step][2] = index_12
        stateVect_12[step][3] = currAction_12
        stateVect_12[step][4] = reward_12
        # Control Simulation
        if currAction_12 > 2:
            phaseToChange = 2
        else:
            phaseToChange = 0
        currTLS_12[0].phases[phaseToChange].duration = Actions[currAction_12]
        currTLS_12[0].phases[phaseToChange].minDur = Actions[currAction_12]
        currTLS_12[0].phases[phaseToChange].maxDur = Actions[currAction_12]
        traci.trafficlight.setCompleteRedYellowGreenDefinition("12", currTLS_12[0])
        # # # For the signal at 13 # # #
        for i in range(4):
            occupancy_13[i] = traci.lane.getLastStepOccupancy(lanes_13[2*i])
        tlsPhase_13 = traci.trafficlight.getPhase("13")
        currTLS_13 = traci.trafficlight.getCompleteRedYellowGreenDefinition("13")
        # Determine the state
        stateVect_13.insert(step, [occupancy_13, tlsPhase_13, 0, 0, 0])
        index_13 = getStateIndex(occupancy_13)
        reward_13 = getStateReturn(occupancy_13)
        # Take Action
        if random.uniform(0, 1) < EPSILON:
            currAction_13 = random.randint(0, 5)
        else:
            currAction_13 = policyPi_13[index_13]
        # Storage
        stateVect_13[step][2] = index_13
        stateVect_13[step][3] = currAction_13
        stateVect_13[step][4] = reward_13
        # Control Simulation
        if currAction_13 > 2:
            phaseToChange = 2
        else:
            phaseToChange = 0
        currTLS_13[0].phases[phaseToChange].duration = Actions[currAction_13]
        currTLS_13[0].phases[phaseToChange].minDur = Actions[currAction_13]
        currTLS_13[0].phases[phaseToChange].maxDur = Actions[currAction_13]
        traci.trafficlight.setCompleteRedYellowGreenDefinition("13", currTLS_13[0])

        # The Update
        if step > 0 and step % 300 == 0:
            episodeStates_6 = [[]]
            episodeStates_7 = [[]]
            episodeStates_12 = [[]]
            episodeStates_13 = [[]]
            A = 0
            R = 0
            idx = 0
            for t in range(0, 299, 1):
                episodeStates_6.insert(t, stateVect_6[step-299+t])
                episodeStates_7.insert(t, stateVect_7[step-299+t])
                episodeStates_12.insert(t, stateVect_12[step-299+t])
                episodeStates_13.insert(t, stateVect_13[step-299+t])
            for t in range(0, 298, 1):
                # # # For the signal at 6 # # #
                R = episodeStates_6[t][4]
                A = episodeStates_6[t][3]
                idx = episodeStates_6[t][2]
                SPrim = episodeStates_6[t+1]
                idxPrim = SPrim[2]
                QSA_6[idx][A] = QSA_6[idx][A] + ALPHA * (R + (GAMMA * max(QSA_6[idxPrim])) - QSA_6[idx][A])
                policyPi_6[idx] = np.argmax(QSA_6[idx])
                # # # For the signal at  7 # # #
                R = episodeStates_7[t][4]
                A = episodeStates_7[t][3]
                idx = episodeStates_7[t][2]
                SPrim = episodeStates_7[t+1]
                idxPrim = SPrim[2]
                QSA_7[idx][A] = QSA_7[idx][A] + ALPHA * (R + (GAMMA * max(QSA_7[idxPrim])) - QSA_7[idx][A])
                policyPi_7[idx] = np.argmax(QSA_7[idx])
                # # # For the signal at 12 # # #
                R = episodeStates_12[t][4]
                A = episodeStates_12[t][3]
                idx = episodeStates_12[t][2]
                SPrim = episodeStates_12[t+1]
                idxPrim = SPrim[2]
                QSA_12[idx][A] = QSA_12[idx][A] + ALPHA * (R + (GAMMA * max(QSA_12[idxPrim])) - QSA_12[idx][A])
                policyPi_12[idx] = np.argmax(QSA_12[idx])
                # # # For the signal at 13 # # #
                R = episodeStates_13[t][4]
                A = episodeStates_13[t][3]
                idx = episodeStates_13[t][2]
                SPrim = episodeStates_13[t+1]
                idxPrim = SPrim[2]
                QSA_13[idx][A] = QSA_13[idx][A] + ALPHA * (R + (GAMMA * max(QSA_13[idxPrim])) - QSA_13[idx][A])
                policyPi_13[idx] = np.argmax(QSA_13[idx])

        step += 1
        # if step == 1:
        #     print(lanes)
        #     break
        curWaittime = 0
        for i in range(16):
            curWaittime += traci.lane.getWaitingTime(lanes[2*i])
        totWaittimeSoFar = totWaittimeSoFar + curWaittime
        avgTime.append(totWaittimeSoFar/step)

        currVehsHere = traci.simulation.getArrivedNumber()
        totVehsSoFar = totVehsSoFar + currVehsHere
        numVehs.append(totVehsSoFar)
    traci.close()
    sys.stdout.flush()
    return step, avgTime, numVehs


def runQFA():
    # Initializations for the signal at 6
    stateVect_6 = [[]]
    QSA_6 = np.random.rand(6, 1)
    phiSA_6 = [[0]*13 for x in range(6)]
    for i in range(6):
        phiSA_6[i][12] = 1
    theta_6 = np.array(np.random.rand(13, 1))
    currTLS_6 = []
    currAction_6 = 0
    reward_6 = 0
    occupancy_6 = [0]*4

    # Initializations for the signal at 7
    stateVect_7 = [[]]
    QSA_7 = np.random.rand(6, 1)
    phiSA_7 = [[0]*13 for x in range(6)]
    for i in range(6):
        phiSA_7[i][12] = 1
    theta_7 = np.array(np.random.rand(13, 1))
    currTLS_7 = []
    currAction_7 = 0
    reward_7 = 0
    occupancy_7 = [0]*4

    # Initializations for the signal at 12
    stateVect_12 = [[]]
    QSA_12 = np.random.rand(6, 1)
    phiSA_12 = [[0]*13 for x in range(6)]
    for i in range(6):
        phiSA_12[i][12] = 1
    theta_12 = np.array(np.random.rand(13, 1))
    currTLS_12 = []
    currAction_12 = 0
    reward_12 = 0
    occupancy_12 = [0]*4

    # Initializations for the signal at 6
    stateVect_13 = [[]]
    QSA_13 = np.random.rand(6, 1)
    phiSA_13 = [[0]*13 for x in range(6)]
    for i in range(6):
        phiSA_13[i][12] = 1
    theta_13 = np.array(np.random.rand(13, 1))
    currTLS_13 = []
    currAction_13 = 0
    reward_13 = 0
    occupancy_13 = [0]*4

    # Initialization for the experiment
    Actions = [20, 25, 30, 20, 25, 30]
    avgTime = []
    numVehs = []
    EPSILON = 0.20
    ALPHA = 0.05
    GAMMA = 0.9
    step = 0
    totWaittimeSoFar = 0
    curWaittime = 0
    totVehsSoFar = 0
    currVehsHere = 0

    # Action!
    lanes_6 = traci.trafficlight.getControlledLanes("6")
    lanes_7 = traci.trafficlight.getControlledLanes("7")
    lanes_12 = traci.trafficlight.getControlledLanes("12")
    lanes_13 = traci.trafficlight.getControlledLanes("13")
    lanes = lanes_6 + lanes_7 + lanes_12 + lanes_13
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        # # # For the signal at 6 # # #
        for i in range(4):
            occupancy_6[i] = traci.lane.getLastStepOccupancy(lanes_6[2*i])
        tlsPhase_6 = traci.trafficlight.getPhase("6")
        currTLS_6 = traci.trafficlight.getCompleteRedYellowGreenDefinition("6")
        # Determine the state
        lenFeature_6 = (occupancy_6[0] + occupancy_6[1] + occupancy_6[2] + occupancy_6[3]) * 1.0 / 4
        timeFeature_6 = getSwitchTime(tlsPhase_6, currTLS_6, traci.simulation.getTime(), '6')
        reward_6 = -1.0 * (lenFeature_6 + timeFeature_6)
        # Take epsilon-greedy action
        if random.uniform(0, 1) < EPSILON:
            currAction_6 = random.randint(0, 5)
        else:
            currAction_6 = np.argmax(QSA_6)
        # Storage
        phiSA_6[currAction_6][currAction_6*2] = lenFeature_6
        phiSA_6[currAction_6][currAction_6*2 + 1] = timeFeature_6
        stateVect_6.insert(step, [lenFeature_6, timeFeature_6, 0, 0])
        stateVect_6[step][3] = currAction_6
        stateVect_6[step][2] = reward_6
        # Control simulation
        if currAction_6 > 2:
            phaseToChange = 2
        else:
            phaseToChange = 0
        currTLS_6[0].phases[phaseToChange].duration = Actions[currAction_6]
        currTLS_6[0].phases[phaseToChange].minDur = Actions[currAction_6]
        currTLS_6[0].phases[phaseToChange].maxDur = Actions[currAction_6]
        traci.trafficlight.setCompleteRedYellowGreenDefinition("6", currTLS_6[0])

        # # # For the signal at 7 # # #
        for i in range(4):
            occupancy_7[i] = traci.lane.getLastStepOccupancy(lanes_7[2*i])
        tlsPhase_7 = traci.trafficlight.getPhase("7")
        currTLS_7 = traci.trafficlight.getCompleteRedYellowGreenDefinition("7")
        # Determine the state
        lenFeature_7 = (occupancy_7[0] + occupancy_7[1] + occupancy_7[2] + occupancy_7[3]) * 1.0 / 4
        timeFeature_7 = getSwitchTime(tlsPhase_7, currTLS_7, traci.simulation.getTime(), '7')
        reward_7 = -1.0 * (lenFeature_7 + timeFeature_7)
        # Take epsilon-greedy action
        if random.uniform(0, 1) < EPSILON:
            currAction_7 = random.randint(0, 5)
        else:
            currAction_7 = np.argmax(QSA_7)
        # Storage
        phiSA_7[currAction_7][currAction_7*2] = lenFeature_7
        phiSA_7[currAction_7][currAction_7*2 + 1] = timeFeature_7
        stateVect_7.insert(step, [lenFeature_7, timeFeature_7, 0, 0])
        stateVect_7[step][3] = currAction_7
        stateVect_7[step][2] = reward_7
        # Control simulation
        if currAction_7 > 2:
            phaseToChange = 2
        else:
            phaseToChange = 0
        currTLS_7[0].phases[phaseToChange].duration = Actions[currAction_7]
        currTLS_7[0].phases[phaseToChange].minDur = Actions[currAction_7]
        currTLS_7[0].phases[phaseToChange].maxDur = Actions[currAction_7]
        traci.trafficlight.setCompleteRedYellowGreenDefinition("7", currTLS_7[0])

        # # # For the signal at 12 # # #
        for i in range(4):
            occupancy_12[i] = traci.lane.getLastStepOccupancy(lanes_12[2*i])
        tlsPhase_12 = traci.trafficlight.getPhase("12")
        currTLS_12 = traci.trafficlight.getCompleteRedYellowGreenDefinition("12")
        # Determine the state
        lenFeature_12 = (occupancy_12[0] + occupancy_12[1] + occupancy_12[2] + occupancy_12[3]) * 1.0 / 4
        timeFeature_12 = getSwitchTime(tlsPhase_12, currTLS_12, traci.simulation.getTime(), '12')
        reward_12 = -1.0 * (lenFeature_12 + timeFeature_12)
        # Take epsilon-greedy action
        if random.uniform(0, 1) < EPSILON:
            currAction_12 = random.randint(0, 5)
        else:
            currAction_12 = np.argmax(QSA_12)
        # Storage
        phiSA_12[currAction_12][currAction_12*2] = lenFeature_12
        phiSA_12[currAction_12][currAction_12*2 + 1] = timeFeature_12
        stateVect_12.insert(step, [lenFeature_12, timeFeature_12, 0, 0])
        stateVect_12[step][3] = currAction_12
        stateVect_12[step][2] = reward_12
        # Control simulation
        if currAction_12 > 2:
            phaseToChange = 2
        else:
            phaseToChange = 0
        currTLS_12[0].phases[phaseToChange].duration = Actions[currAction_12]
        currTLS_12[0].phases[phaseToChange].minDur = Actions[currAction_12]
        currTLS_12[0].phases[phaseToChange].maxDur = Actions[currAction_12]
        traci.trafficlight.setCompleteRedYellowGreenDefinition("12", currTLS_12[0])

        # # # For the signal at 13 # # #
        for i in range(4):
            occupancy_13[i] = traci.lane.getLastStepOccupancy(lanes_13[2*i])
        tlsPhase_13 = traci.trafficlight.getPhase("13")
        currTLS_13 = traci.trafficlight.getCompleteRedYellowGreenDefinition("13")
        # Determine the state
        lenFeature_13 = (occupancy_13[0] + occupancy_13[1] + occupancy_13[2] + occupancy_13[3]) * 1.0 / 4
        timeFeature_13 = getSwitchTime(tlsPhase_13, currTLS_13, traci.simulation.getTime(), '13')
        reward_13 = -1.0 * (lenFeature_13 + timeFeature_13)
        # Take epsilon-greedy action
        if random.uniform(0, 1) < EPSILON:
            currAction_13 = random.randint(0, 5)
        else:
            currAction_13 = np.argmax(QSA_13)
        # Storage
        phiSA_13[currAction_13][currAction_13*2] = lenFeature_13
        phiSA_13[currAction_13][currAction_13*2 + 1] = timeFeature_13
        stateVect_13.insert(step, [lenFeature_13, timeFeature_13, 0, 0])
        stateVect_13[step][3] = currAction_13
        stateVect_13[step][2] = reward_13
        # Control simulation
        if currAction_13 > 2:
            phaseToChange = 2
        else:
            phaseToChange = 0
        currTLS_13[0].phases[phaseToChange].duration = Actions[currAction_13]
        currTLS_13[0].phases[phaseToChange].minDur = Actions[currAction_13]
        currTLS_13[0].phases[phaseToChange].maxDur = Actions[currAction_13]
        traci.trafficlight.setCompleteRedYellowGreenDefinition("13", currTLS_13[0])
        # # # # # # # #
        #  The Update #
        if step > 0 and step % 300 == 0:
            episodeStates_6 = [[]]
            episodeStates_7 = [[]]
            episodeStates_12 = [[]]
            episodeStates_13 = [[]]
            A = 0
            R = 0
            for t in range(0, 299, 1):
                episodeStates_6.insert(t, stateVect_6[step-299+t])
                episodeStates_7.insert(t, stateVect_7[step-299+t])
                episodeStates_12.insert(t, stateVect_12[step-299+t])
                episodeStates_13.insert(t, stateVect_13[step-299+t])
            for t in range(0, 298, 1):
                # # # For the signal at 6 # # #
                R = episodeStates_6[t][2]
                A = episodeStates_6[t][3]
                SPrim = episodeStates_6[t+1]
                QSA_6 = np.matmul(phiSA_6, theta_6)
                QdashSA_6 = getQdash(phiSA_6, SPrim, theta_6)
                gradWqsa_6 = [0]*13
                for i in range(13):
                    for j in range(6):
                        gradWqsa_6[i] = gradWqsa_6[i] + phiSA_6[j][i]
                err_6 = R + GAMMA * max(QdashSA_6) - QSA_6[A]
                const_6 = ALPHA * err_6
                for i in range(13):
                    theta_6[i] = theta_6[i] + const_6 * gradWqsa_6[i]
                # # # For the signal at  7 # # #
                R = episodeStates_7[t][2]
                A = episodeStates_7[t][3]
                Sprime_7 = episodeStates_7[t+1]
                QSA_7 = np.matmul(phiSA_7, theta_7)
                QdashSA_7 = getQdash(phiSA_7, Sprime_7, theta_7)
                gradWqsa_7 = [0]*13
                for i in range(13):
                    for j in range(6):
                        gradWqsa_7[i] = gradWqsa_7[i] + phiSA_7[j][i]
                err_7 = R + GAMMA * max(QdashSA_7) - QSA_7[A]
                const_7 = ALPHA * err_7
                for i in range(13):
                    theta_7[i] = theta_7[i] + const_7 * gradWqsa_7[i]
                # # # For the signal at  12 # # #
                R = episodeStates_12[t][2]
                A = episodeStates_12[t][3]
                Sprime_12 = episodeStates_12[t+1]
                QSA_12 = np.matmul(phiSA_12, theta_12)
                QdashSA_12 = getQdash(phiSA_12, Sprime_12, theta_12)
                gradWqsa_12 = [0]*13
                for i in range(13):
                    for j in range(6):
                        gradWqsa_12[i] = gradWqsa_12[i] + phiSA_12[j][i]
                err_12 = R + GAMMA * max(QdashSA_12) - QSA_12[A]
                const_12 = ALPHA * err_12
                for i in range(13):
                    theta_12[i] = theta_12[i] + const_12 * gradWqsa_12[i]
                # # # For the signal at  13 # # #
                R = episodeStates_13[t][2]
                A = episodeStates_13[t][3]
                Sprime_13 = episodeStates_13[t+1]
                QSA_13 = np.matmul(phiSA_13, theta_13)
                QdashSA_13 = getQdash(phiSA_13, Sprime_13, theta_13)
                gradWqsa_13 = [0]*13
                for i in range(13):
                    for j in range(6):
                        gradWqsa_13[i] = gradWqsa_13[i] + phiSA_13[j][i]
                err_13 = R + GAMMA * max(QdashSA_13) - QSA_13[A]
                const_13 = ALPHA * err_13
                for i in range(13):
                    theta_13[i] = theta_13[i] + const_13 * gradWqsa_13[i]

        step += 1

        curWaittime = 0
        for i in range(16):
            curWaittime += traci.lane.getWaitingTime(lanes[2*i])
        totWaittimeSoFar = totWaittimeSoFar + curWaittime
        avgTime.append(totWaittimeSoFar/step)

        currVehsHere = traci.simulation.getArrivedNumber()
        totVehsSoFar = totVehsSoFar + currVehsHere
        numVehs.append(totVehsSoFar)
    traci.close()
    sys.stdout.flush()
    return step, avgTime, numVehs


def runFixed(dur1, dur2):
    avgTime = []
    numVehs = []
    avgTime.clear()
    numVehs.clear()
    step = 0
    totWaittimeSoFar = 0
    totVehsSoFar = 0
    currTLS = traci.trafficlight.getCompleteRedYellowGreenDefinition("6")
    currTLS[0].phases[0].duration = dur1  # * 1000
    currTLS[0].phases[0].minDur = dur1  # * 1000
    currTLS[0].phases[0].maxDur = dur1  # * 1000

    currTLS[0].phases[2].duration = dur2  # * 1000
    currTLS[0].phases[2].minDur = dur2  # * 1000
    currTLS[0].phases[2].maxDur = dur2  # * 1000
    traci.trafficlight.setCompleteRedYellowGreenDefinition("6", currTLS[0])

    currTLS = traci.trafficlight.getCompleteRedYellowGreenDefinition("7")
    currTLS[0].phases[0].duration = dur2  # * 1000
    currTLS[0].phases[0].minDur = dur2  # * 1000
    currTLS[0].phases[0].maxDur = dur2  # * 1000

    currTLS[0].phases[2].duration = dur1  # * 1000
    currTLS[0].phases[2].minDur = dur1  # * 1000
    currTLS[0].phases[2].maxDur = dur1  # * 1000
    traci.trafficlight.setCompleteRedYellowGreenDefinition("7", currTLS[0])

    currTLS = traci.trafficlight.getCompleteRedYellowGreenDefinition("12")
    currTLS[0].phases[0].duration = dur2  # * 1000
    currTLS[0].phases[0].minDur = dur2  # * 1000
    currTLS[0].phases[0].maxDur = dur2  # * 1000

    currTLS[0].phases[2].duration = dur1  # * 1000
    currTLS[0].phases[2].minDur = dur1  # * 1000
    currTLS[0].phases[2].maxDur = dur1  # * 1000
    traci.trafficlight.setCompleteRedYellowGreenDefinition("12", currTLS[0])

    currTLS = traci.trafficlight.getCompleteRedYellowGreenDefinition("13")
    currTLS[0].phases[0].duration = dur1  # * 1000
    currTLS[0].phases[0].minDur = dur1  # * 1000
    currTLS[0].phases[0].maxDur = dur1  # * 1000

    currTLS[0].phases[2].duration = dur2  # * 1000
    currTLS[0].phases[2].minDur = dur2  # * 1000
    currTLS[0].phases[2].maxDur = dur2  # * 1000
    traci.trafficlight.setCompleteRedYellowGreenDefinition("13", currTLS[0])

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1
        lanes6 = traci.trafficlight.getControlledLanes("6")
        lanes7 = traci.trafficlight.getControlledLanes("7")
        lanes12 = traci.trafficlight.getControlledLanes("12")
        lanes13 = traci.trafficlight.getControlledLanes("13")
        lanes = lanes6 + lanes7 + lanes12 + lanes13
        curWaittime = 0
        for i in range(16):
            curWaittime += traci.lane.getWaitingTime(lanes[2*i])
        # if step == 1:
        #     print(lanes, sep='\n')
        #     break
        totWaittimeSoFar = totWaittimeSoFar + curWaittime
        avgTime.append(totWaittimeSoFar/step)

        currVehsHere = traci.simulation.getArrivedNumber()
        totVehsSoFar = totVehsSoFar + currVehsHere
        numVehs.append(totVehsSoFar)
        # print(totWaittimeSoFar, curWaittime)
    traci.close()
    # plt.figure()
    # plt.plot(avgTime, label='Fixed Duration: 20/20', ls=':', c='b')
    # plt.plot(numVehs, label='Fixed Duration: 20/20', ls=':', c='b')
    # plt.show()
    sys.stdout.flush()
    return step, avgTime, numVehs


def seconds(x, pos):
    return int(x*1e-3)


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()
    # this script has been called from the command line. It will start sumo as a server, then connect and run
    if options.gui:
        sumoBinary = checkBinary('sumo-gui')
    else:
        sumoBinary = checkBinary('sumo')

    traci.start([sumoBinary, "-c", "data/2x2.sumocfg"])
    step1, avgTime1, numVehs1 = runFixed(20, 20)

    traci.start([sumoBinary, "-c", "data/2x2.sumocfg"])
    step2, avgTime2, numVehs2 = runFixed(25, 25)

    traci.start([sumoBinary, "-c", "data/2x2.sumocfg"])
    step3, avgTime3, numVehs3 = runFixed(20, 25)

    traci.start([sumoBinary, "-c", "data/2x2.sumocfg"])
    step4, avgTime4, numVehs4 = runFixed(25, 20)

    traci.start([sumoBinary, "-c", "data/2x2.sumocfg"])
    step5, avgTime5, numVehs5 = runQFS()

    traci.start([sumoBinary, "-c", "data/2x2.sumocfg"])
    step6, avgTime6, numVehs6 = runQFA()

    # formatter = FuncFormatter(seconds)
    plt.figure()
    # # fig, ax = plt.subplots()
    # # ax.yaxis.set_major_formatter(formatter)
    plt.plot(avgTime1, label='Fixed Duration: 20/20', ls=':', c='b')
    plt.plot(avgTime2, label='Fixed Duration: 25/25', ls='-.', c='c')
    plt.plot(avgTime3, label='Fixed Duration: 20/25', ls='--', c='m')
    plt.plot(avgTime4, label='Fixed Duration: 25/20', dashes=[5, 2, 1, 2, 1, 2], ls='-', c='r')
    plt.plot(avgTime5, label='Dynamic Duration: Q-learning with FS', dashes=[5, 2, 20, 2], ls='-', c='g')
    plt.plot(avgTime6, label='Dynamic Duration: Q-learning with FA', lw=2, ls='solid', c='k')
    plt.legend(loc='lower right')
    plt.xlabel('Simulation Steps')
    plt.ylabel('Average Waiting Time')
    plt.title('Traffic for 2x2 grid: Average Waiting Time')
    # plt.savefig('./Results/FixedVsQLFA/QFA_AWT.eps', bbox_inches='tight', format='eps', dpi=1200)
    # plt.savefig('./Results/FixedVsQLFA/QFA_AWT.svg', bbox_inches='tight', format='svg', dpi=1200)
    interactive(True)
    plt.show()

    plt.figure()
    plt.plot(numVehs1, label='Fixed Duration: 20/20', ls=':', c='b')
    plt.plot(numVehs2, label='Fixed Duration: 25/25', ls='-.', c='c')
    plt.plot(numVehs3, label='Fixed Duration: 20/25', ls='--', c='m')
    plt.plot(numVehs4, label='Fixed Duration: 25/20', dashes=[5, 2, 1, 2, 1, 2], ls='-', c='r')
    plt.plot(numVehs5, label='Dynamic Duration: Q-learning with FS', dashes=[5, 2, 20, 2], ls='-', c='g')
    plt.plot(numVehs6, label='Dynamic Duration: Q-learning with FA', lw=2, ls='solid', c='k')
    plt.legend(loc='lower right')
    plt.xlabel('Simulation Steps')
    plt.ylabel('Total # vehicles arrived')
    plt.title('Traffic for 2x2 grid: Number of Vehicles')
    # plt.savefig('./Results/FixedVsQLFA/QFA_TNV.eps', bbox_inches='tight', format='eps', dpi=1200)
    # plt.savefig('./Results/FixedVsQLFA/QFA_TNV.svg', bbox_inches='tight', format='svg', dpi=1200)
    interactive(False)
    plt.show()
