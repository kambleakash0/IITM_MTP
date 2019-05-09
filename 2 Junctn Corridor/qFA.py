#!/usr/bin/python

from __future__ import absolute_import
from __future__ import print_function

import sumoPath
import sys
import optparse
import subprocess
import random
import numpy as np
from matplotlib import interactive
from matplotlib.ticker import FuncFormatter
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
    # Initializations for the left_sig
    stateVect_left = [[]]
    policyPi_left = []
    for i in range(0, 80, 1):
        policyPi_left.insert(i, random.randint(0, 5))
    QSA_left = np.random.rand(81, 6)
    currTLS_left = []
    currAction_left = 0
    reward_left = 0
    occupancy_left = [0]*4

    # Initializations for the right_sig
    stateVect_right = [[]]
    policyPi_right = []
    for i in range(0, 80, 1):
        policyPi_right.insert(i, random.randint(0, 5))
    QSA_right = np.random.rand(81, 6)
    currTLS_right = []
    currAction_right = 0
    reward_right = 0
    occupancy_right = [0]*4

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
    leftLanes = traci.trafficlight.getControlledLanes("left_sig")
    rightLanes = traci.trafficlight.getControlledLanes("right_sig")
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        # # # For the left_sig # # #
        for i in range(4):
            occupancy_left[i] = traci.lane.getLastStepOccupancy(leftLanes[2*i])
        tlsPhase_left = traci.trafficlight.getPhase("left_sig")
        currTLS_left = traci.trafficlight.getCompleteRedYellowGreenDefinition("left_sig")
        # Determine the state
        stateVect_left.insert(step, [occupancy_left, tlsPhase_left, 0, 0, 0])
        index_left = getStateIndex(occupancy_left)
        reward_left = getStateReturn(occupancy_left)
        # Take Action
        if random.uniform(0, 1) < EPSILON:
            currAction_left = random.randint(0, 5)
        else:
            currAction_left = policyPi_left[index_left]
        # Storage
        stateVect_left[step][2] = index_left
        stateVect_left[step][3] = currAction_left
        stateVect_left[step][4] = reward_left
        # Control Simulation
        if currAction_left > 2:
            phaseToChange = 2
        else:
            phaseToChange = 0
        currTLS_left[0].phases[phaseToChange].duration = Actions[currAction_left]
        currTLS_left[0].phases[phaseToChange].minDur = Actions[currAction_left]
        currTLS_left[0].phases[phaseToChange].maxDur = Actions[currAction_left]
        traci.trafficlight.setCompleteRedYellowGreenDefinition("left_sig", currTLS_left[0])
        # # # For the right_sig # # #
        for i in range(4):
            occupancy_right[i] = traci.lane.getLastStepOccupancy(rightLanes[2*i])
        tlsPhase_right = traci.trafficlight.getPhase("right_sig")
        currTLS_right = traci.trafficlight.getCompleteRedYellowGreenDefinition("right_sig")
        # Determine the state
        stateVect_right.insert(step, [occupancy_right, tlsPhase_right, 0, 0, 0])
        index_right = getStateIndex(occupancy_right)
        reward_right = getStateReturn(occupancy_right)
        # Take Action
        if random.uniform(0, 1) < EPSILON:
            currAction_right = random.randint(0, 5)
        else:
            currAction_right = policyPi_right[index_right]
        # Storage
        stateVect_right[step][2] = index_right
        stateVect_right[step][3] = currAction_right
        stateVect_right[step][4] = reward_right
        # Control Simulation
        if currAction_right > 2:
            phaseToChange = 2
        else:
            phaseToChange = 0
        currTLS_right[0].phases[phaseToChange].duration = Actions[currAction_right]
        currTLS_right[0].phases[phaseToChange].minDur = Actions[currAction_right]
        currTLS_right[0].phases[phaseToChange].maxDur = Actions[currAction_right]
        traci.trafficlight.setCompleteRedYellowGreenDefinition("right_sig", currTLS_right[0])

        # The Update
        if step > 0 and step % 300 == 0:
            # # # For the left_sig # # #
            episodeStates_left = [[]]
            A_left = 0
            R_left = 0
            index_left = 0
            # # # For the right_sig # # #
            episodeStates_right = [[]]
            A_right = 0
            R_right = 0
            index_right = 0
            for t in range(0, 299, 1):
                episodeStates_left.insert(t, stateVect_left[step-299+t])
                episodeStates_right.insert(t, stateVect_right[step-299+t])
            for t in range(0, 298, 1):
                # # # For the left_sig # # #
                R_left = episodeStates_left[t][4]
                A_left = episodeStates_left[t][3]
                index_left = episodeStates_left[t][2]
                Sprime = episodeStates_left[t+1]
                indexPrime = Sprime[2]
                QSA_left[index_left][A_left] = QSA_left[index_left][A_left] + ALPHA * (R_left + (GAMMA * max(QSA_left[indexPrime])) - QSA_left[index_left][A_left])
                policyPi_left[index_left] = np.argmax(QSA_left[index_left])
                # # # For the right_sig # # #
                R_right = episodeStates_right[t][4]
                A_right = episodeStates_right[t][3]
                index_right = episodeStates_right[t][2]
                Sprime = episodeStates_right[t+1]
                indexPrime = Sprime[2]
                QSA_right[index_right][A_right] = QSA_right[index_right][A_right] + ALPHA * (R_right + (GAMMA * max(QSA_right[indexPrime])) - QSA_right[index_right][A_right])
                policyPi_right[index_right] = np.argmax(QSA_right[index_right])

        step += 1
        curWaittime = traci.edge.getWaitingTime('slls') + traci.edge.getWaitingTime('wls') + traci.edge.getWaitingTime('nlls') + traci.edge.getWaitingTime('rsls') + traci.edge.getWaitingTime('lsrs') + traci.edge.getWaitingTime('srrs') + traci.edge.getWaitingTime('ers') + traci.edge.getWaitingTime('nrrs')
        totWaittimeSoFar = totWaittimeSoFar + curWaittime
        avgTime.append(totWaittimeSoFar/step)

        currVehsHere = traci.simulation.getArrivedNumber()
        totVehsSoFar = totVehsSoFar + currVehsHere
        numVehs.append(totVehsSoFar)
    traci.close()
    sys.stdout.flush()
    return step, avgTime, numVehs


def runQFA():
    # Initializations for the left_sig
    stateVect_left = [[]]
    QSA_left = np.random.rand(6, 1)
    phiSA_left = [[0]*13 for x in range(6)]
    for i in range(6):
        phiSA_left[i][12] = 1
    theta_left = np.array(np.random.rand(13, 1))
    currTLS_left = []
    currAction_left = 0
    reward_left = 0
    occupancy_left = [0]*4

    # Initializations for the right_sig
    stateVect_right = [[]]
    QSA_right = np.random.rand(6, 1)
    phiSA_right = [[0]*13 for x in range(6)]
    for i in range(6):
        phiSA_right[i][12] = 1
    theta_right = np.array(np.random.rand(13, 1))
    currTLS_right = []
    currAction_right = 0
    reward_right = 0
    occupancy_right = [0]*4

    Actions = [20, 25, 30, 20, 25, 30]
    avgTime = []
    numVehs = []

    EPSILON = 0.20
    ALPHA = 0.05
    GAMMA = 0.9
    step = 0

    leftLanes = traci.trafficlight.getControlledLanes("left_sig")
    rightLanes = traci.trafficlight.getControlledLanes("right_sig")
    # lanelist = left + right

    totWaittimeSoFar = 0
    curWaittime = 0
    totVehsSoFar = 0
    currVehsHere = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        # # # For the left_sig # # #
        for i in range(4):
            occupancy_left[i] = traci.lane.getLastStepOccupancy(leftLanes[2*i])
        tlsPhase_left = traci.trafficlight.getPhase("left_sig")
        currTLS_left = traci.trafficlight.getCompleteRedYellowGreenDefinition("left_sig")
        # Determine the state
        lenFeature_left = (occupancy_left[0] + occupancy_left[1] + occupancy_left[2] + occupancy_left[3]) * 1.0 / 4
        timeFeature_left = getSwitchTime(tlsPhase_left, currTLS_left, traci.simulation.getTime(), 'left_sig')
        reward_left = -1.0 * (lenFeature_left + timeFeature_left)
        # Take epsilon-greedy action
        if random.uniform(0, 1) < EPSILON:
            currAction_left = random.randint(0, 5)
        else:
            currAction_left = np.argmax(QSA_left)
        # Storage
        phiSA_left[currAction_left][currAction_left*2] = lenFeature_left
        phiSA_left[currAction_left][currAction_left*2 + 1] = timeFeature_left
        stateVect_left.insert(step, [lenFeature_left, timeFeature_left, 0, 0])
        stateVect_left[step][3] = currAction_left
        stateVect_left[step][2] = reward_left
        # Control simulation
        if currAction_left > 2:
            phaseToChange = 2
        else:
            phaseToChange = 0
        currTLS_left[0].phases[phaseToChange].duration = Actions[currAction_left]
        currTLS_left[0].phases[phaseToChange].minDur = Actions[currAction_left]
        currTLS_left[0].phases[phaseToChange].maxDur = Actions[currAction_left]
        traci.trafficlight.setCompleteRedYellowGreenDefinition("left_sig", currTLS_left[0])

        # # # For the right_sig # # #
        for i in range(4):
            occupancy_right[i] = traci.lane.getLastStepOccupancy(rightLanes[2*i])
        tlsPhase_right = traci.trafficlight.getPhase("right_sig")
        currTLS_right = traci.trafficlight.getCompleteRedYellowGreenDefinition("right_sig")
        # Determine the state
        lenFeature_right = (occupancy_right[0] + occupancy_right[1] + occupancy_right[2] + occupancy_right[3]) * 1.0 / 4
        timeFeature_right = getSwitchTime(tlsPhase_right, currTLS_right, traci.simulation.getTime(), 'right_sig')
        reward_right = -1.0 * (lenFeature_right + timeFeature_right)
        # Take epsilon-greedy action
        if random.uniform(0, 1) < EPSILON:
            currAction_right = random.randint(0, 5)
        else:
            currAction_right = np.argmax(QSA_right)
        # Storage
        phiSA_right[currAction_right][currAction_right*2] = lenFeature_right
        phiSA_right[currAction_right][currAction_right*2 + 1] = timeFeature_right
        stateVect_right.insert(step, [lenFeature_right, timeFeature_right, 0, 0])
        stateVect_right[step][3] = currAction_right
        stateVect_right[step][2] = reward_right
        # Control simulation
        if currAction_left > 2:
            phaseToChange = 2
        else:
            phaseToChange = 0
        currTLS_right[0].phases[phaseToChange].duration = Actions[currAction_right]
        currTLS_right[0].phases[phaseToChange].minDur = Actions[currAction_right]
        currTLS_right[0].phases[phaseToChange].maxDur = Actions[currAction_right]
        traci.trafficlight.setCompleteRedYellowGreenDefinition("right_sig", currTLS_right[0])

        # The Update
        if step > 0 and step % 300 == 0:
            # # # For the left_sig # # #
            episodeStates_left = [[]]
            A_left = 0
            R_left = 0

            # # # For the right_sig # # #
            episodeStates_right = [[]]
            A_right = 0
            R_right = 0

            for t in range(0, 299, 1):
                episodeStates_left.insert(t, stateVect_left[step-299+t])
                episodeStates_right.insert(t, stateVect_right[step-299+t])

            for t in range(0, 298, 1):
                # # # For the left_sig # # #
                R_left = episodeStates_left[t][2]
                A_left = episodeStates_left[t][3]
                Sprime_left = episodeStates_left[t+1]
                QSA_left = np.matmul(phiSA_left, theta_left)
                QdashSA_left = getQdash(phiSA_left, Sprime_left, theta_left)
                gradWqsa_left = [0]*13
                for i in range(13):
                    for j in range(6):
                        gradWqsa_left[i] = gradWqsa_left[i] + phiSA_left[j][i]
                err_left = R_left + GAMMA * max(QdashSA_left) - QSA_left[A_left]
                # err_left = R_left + GAMMA *
                const_left = ALPHA * err_left
                for i in range(13):
                    theta_left[i] = theta_left[i] + const_left * gradWqsa_left[i]

                # # # For the right_sig # # #
                R_right = episodeStates_right[t][2]
                A_right = episodeStates_right[t][3]
                Sprime_right = episodeStates_right[t+1]
                QSA_right = np.matmul(phiSA_right, theta_right)
                QdashSA_right = getQdash(phiSA_right, Sprime_right, theta_right)
                gradWqsa_right = [0]*13
                for i in range(13):
                    for j in range(6):
                        gradWqsa_right[i] = gradWqsa_right[i] + phiSA_right[j][i]
                err_right = R_right + GAMMA * max(QdashSA_right) - QSA_right[A_right]
                const_right = ALPHA * err_right
                for i in range(13):
                    theta_right[i] = theta_right[i] + const_right * gradWqsa_right[i]

        step += 1

        curWaittime = traci.edge.getWaitingTime('slls') + traci.edge.getWaitingTime('wls') + traci.edge.getWaitingTime('nlls') + traci.edge.getWaitingTime('rsls') + traci.edge.getWaitingTime('lsrs') + traci.edge.getWaitingTime('srrs') + traci.edge.getWaitingTime('ers') + traci.edge.getWaitingTime('nrrs')
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
    currTLS = traci.trafficlight.getCompleteRedYellowGreenDefinition("left_sig")
    currTLS[0].phases[0].duration = dur1  # * 1000
    currTLS[0].phases[0].minDur = dur1  # * 1000
    currTLS[0].phases[0].maxDur = dur1  # * 1000

    currTLS[0].phases[2].duration = dur2  # * 1000
    currTLS[0].phases[2].minDur = dur2  # * 1000
    currTLS[0].phases[2].maxDur = dur2  # * 1000
    traci.trafficlight.setCompleteRedYellowGreenDefinition("left_sig", currTLS[0])

    currTLS = traci.trafficlight.getCompleteRedYellowGreenDefinition("right_sig")
    currTLS[0].phases[0].duration = dur1  # * 1000
    currTLS[0].phases[0].minDur = dur1  # * 1000
    currTLS[0].phases[0].maxDur = dur1  # * 1000

    currTLS[0].phases[2].duration = dur2  # * 1000
    currTLS[0].phases[2].minDur = dur2  # * 1000
    currTLS[0].phases[2].maxDur = dur2  # * 1000
    traci.trafficlight.setCompleteRedYellowGreenDefinition("right_sig", currTLS[0])

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1
        curWaittime = traci.edge.getWaitingTime('slls') + traci.edge.getWaitingTime('wls') + traci.edge.getWaitingTime('nlls') + traci.edge.getWaitingTime('rsls') + traci.edge.getWaitingTime('lsrs') + traci.edge.getWaitingTime('srrs') + traci.edge.getWaitingTime('ers') + traci.edge.getWaitingTime('nrrs')
        totWaittimeSoFar = totWaittimeSoFar + curWaittime
        avgTime.append(totWaittimeSoFar/step)

        currVehsHere = traci.simulation.getArrivedNumber()
        totVehsSoFar = totVehsSoFar + currVehsHere
        numVehs.append(totVehsSoFar)
    traci.close()
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

    traci.start([sumoBinary, "-c", "data/2sig.sumocfg"])
    step1, avgTime1, numVehs1 = runFixed(20, 20)

    traci.start([sumoBinary, "-c", "data/2sig.sumocfg"])
    step2, avgTime2, numVehs2 = runFixed(25, 25)

    traci.start([sumoBinary, "-c", "data/2sig.sumocfg"])
    step3, avgTime3, numVehs3 = runFixed(20, 25)

    traci.start([sumoBinary, "-c", "data/2sig.sumocfg"])
    step4, avgTime4, numVehs4 = runFixed(25, 20)

    traci.start([sumoBinary, "-c", "data/2sig.sumocfg"])
    step5, avgTime5, numVehs5 = runQFS()

    traci.start([sumoBinary, "-c", "data/2sig.sumocfg"])
    step6, avgTime6, numVehs6 = runQFA()

    formatter = FuncFormatter(seconds)
    plt.figure()
    # fig, ax = plt.subplots()
    # ax.yaxis.set_major_formatter(formatter)
    plt.plot(avgTime1, label='Fixed Duration: 20/20', ls=':', c='b')
    plt.plot(avgTime2, label='Fixed Duration: 25/25', ls='-.', c='c')
    plt.plot(avgTime3, label='Fixed Duration: 20/25', ls='--', c='m')
    plt.plot(avgTime4, label='Fixed Duration: 25/20', dashes=[5, 2, 1, 2, 1, 2], ls='-', c='r')
    plt.plot(avgTime5, label='Dynamic Duration: Q-learning with FS', dashes=[5, 2, 20, 2], ls='-', c='g')
    plt.plot(avgTime6, label='Dynamic Duration: Q-learning with FA', lw=2, ls='solid', c='k')
    plt.legend(loc='lower right')
    plt.xlabel('Simulation Steps')
    plt.ylabel('Average Waiting Time')
    plt.title('Traffic for 2 Junction Corridor: Average Waiting Time')
    plt.savefig('./Results/FixedVsQLFA/QFA_AWT.eps', bbox_inches='tight', format='eps', dpi=1200)
    plt.savefig('./Results/FixedVsQLFA/QFA_AWT.svg', bbox_inches='tight', format='svg', dpi=1200)
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
    plt.title('Traffic for 2 Junction Corridor: Number of Vehicles')
    plt.savefig('./Results/FixedVsQLFA/QFA_TNV.eps', bbox_inches='tight', format='eps', dpi=1200)
    plt.savefig('./Results/FixedVsQLFA/QFA_TNV.svg', bbox_inches='tight', format='svg', dpi=1200)
    interactive(False)
    plt.show()
