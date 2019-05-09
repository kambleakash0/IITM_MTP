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
    idx = state[0]*27 + state[1]*9 + state[2]*3 + state[3]
    return idx


def getStateReturn(state):
    reward = 0
    for i in range(4):
        reward = reward - state[i]
    return reward


def getSwitchTime(tlsPhase, currTLS, tNow):
    if tlsPhase == 0 or tlsPhase == 2:
        elapsedSinceLastSwitch = tNow - (traci.trafficlight.getNextSwitch("center") - traci.trafficlight.getPhaseDuration("center"))
        total = traci.trafficlight.getPhaseDuration("center") + currTLS[0].phases[tlsPhase+1].duration
    else:
        elapsedSinceLastSwitch = tNow - (traci.trafficlight.getNextSwitch("center") - traci.trafficlight.getPhaseDuration("center") - currTLS[0].phases[tlsPhase-1].duration)
        total = traci.trafficlight.getPhaseDuration("center") + currTLS[0].phases[tlsPhase-1].duration
    return (elapsedSinceLastSwitch / total)


def getQdash(phiSA, Sprime, theta):
    Actn = Sprime[3]
    lenFeature = Sprime[0]
    timeFeature = Sprime[1]
    # phiDashSA = [[0]*13 for x in range(6)]
    # for i in range(6):
    #     phiDashSA[i][12] = 1
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
    stateVect = [[]]
    policyPi = []
    for i in range(0, 80, 1):
        policyPi.insert(i, random.randint(0, 5))
    QSA = np.random.rand(81, 6)
    currTLS = []
    Actions = [20, 25, 30, 20, 25, 30]
    avgTime = []
    numVehs = []
    avgTime.clear()
    numVehs.clear()
    EPSILON = 0.20
    ALPHA = 0.05
    GAMMA = 0.9
    step = 0
    stateNorth = 0
    stateSouth = 0
    stateEast = 0
    stateWest = 0
    currAction = 0
    reward = 0
    totWaittimeSoFar = 0
    curWaittime = 0
    totVehsSoFar = 0
    currVehsHere = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        # Determine the state
        stateNorth = traci.lane.getLastStepOccupancy('nc_0')
        stateSouth = traci.lane.getLastStepOccupancy('sc_0')
        stateEast = traci.lane.getLastStepOccupancy('ec_0')
        stateWest = traci.lane.getLastStepOccupancy('wc_0')
        tlsPhase = traci.trafficlight.getPhase("center")
        stateVect.insert(step, [getState(stateNorth), getState(stateEast), getState(stateSouth), getState(stateWest), getPhase(tlsPhase), 0, 0, 0])
        index = getStateIndex(stateVect[step])
        reward = getStateReturn(stateVect[step])
        # Take Action
        if random.uniform(0, 1) < EPSILON:
            currAction = random.randint(0, 5)
        else:
            currAction = np.argmax(QSA[index])
        # Storage
        stateVect[step][5] = index
        stateVect[step][6] = currAction
        stateVect[step][7] = reward
        # Control Simulation
        if currAction > 2:
            phaseToChange = 2
        else:
            phaseToChange = 0
        currTLS = traci.trafficlight.getCompleteRedYellowGreenDefinition("center")
        currTLS[0].phases[phaseToChange].duration = Actions[currAction]
        currTLS[0].phases[phaseToChange].minDur = Actions[currAction]
        currTLS[0].phases[phaseToChange].maxDur = Actions[currAction]
        traci.trafficlight.setCompleteRedYellowGreenDefinition("center", currTLS[0])
        # The Update
        if step > 0 and step % 75 == 0:
            episodeStates = [[]]
            index = 0
            A = 0
            R = 0
            for t in range(0, 74, 1):
                episodeStates.insert(t, stateVect[step-74+t])
            for t in range(0, 73, 1):
                S = episodeStates[t]
                R = S[7]
                A = S[6]
                index = S[5]
                Sprime = episodeStates[t+1]
                indexPrime = Sprime[5]
                QSA[index][A] = QSA[index][A] + ALPHA * (R + (GAMMA * max(QSA[indexPrime])) - QSA[index][A])
                policyPi[index] = np.argmax(QSA[index])

        step += 1
        curWaittime = traci.edge.getWaitingTime('nc') + traci.edge.getWaitingTime('ec') + traci.edge.getWaitingTime('sc') + traci.edge.getWaitingTime('wc')
        totWaittimeSoFar = totWaittimeSoFar + curWaittime
        avgTime.append(totWaittimeSoFar/step)

        currVehsHere = traci.simulation.getArrivedNumber()
        totVehsSoFar = totVehsSoFar + currVehsHere
        numVehs.append(totVehsSoFar)
    traci.close()
    sys.stdout.flush()
    return step, avgTime, numVehs


def runQFA():
    stateVect = [[]]
    # policyPi = []
    # for i in range(0, 80, 1):
    #     policyPi.insert(i, random.randint(0, 5))
    QSA = np.random.rand(6, 1)
    # QdashSA = np.array(np.random.rand(1, 6))
    # Returns = [[0]*6 for x in range(81)]
    phiSA = [[0]*13 for x in range(6)]
    for i in range(6):
        phiSA[i][12] = 1
    theta = np.array(np.random.rand(13, 1))
    # thetaDash = np.array(np.random.rand(13, 1))
    currTLS = []
    Actions = [20, 25, 30, 20, 25, 30]
    # Actions = [30, 60, 120, 30, 60, 120]
    avgTime = []
    numVehs = []
    avgTime.clear()
    numVehs.clear()

    EPSILON = 0.20
    ALPHA = 0.05
    GAMMA = 0.9
    step = 0

    stateNorth = 0
    stateSouth = 0
    stateEast = 0
    stateWest = 0

    currAction = 0
    reward = 0
    totWaittimeSoFar = 0
    curWaittime = 0
    totVehsSoFar = 0
    currVehsHere = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        # print(QSA)
        stateNorth = traci.lane.getLastStepOccupancy('nc_0')
        stateSouth = traci.lane.getLastStepOccupancy('sc_0')
        stateEast = traci.lane.getLastStepOccupancy('ec_0')
        stateWest = traci.lane.getLastStepOccupancy('wc_0')
        tlsPhase = traci.trafficlight.getPhase("center")
        currTLS = traci.trafficlight.getCompleteRedYellowGreenDefinition("center")
        # Determine the state
        lenFeature = (getState(stateNorth) + getState(stateEast) + getState(stateSouth) + getState(stateWest))*1.0 / 8
        timeFeature = getSwitchTime(tlsPhase, currTLS, traci.simulation.getTime())
        reward = -1.0 * (lenFeature + timeFeature)
        # Take epsilon-greedy action
        if random.uniform(0, 1) < EPSILON:
            currAction = random.randint(0, 5)
        else:
            currAction = np.argmax(QSA)
        # Storage
        phiSA[currAction][currAction*2] = lenFeature
        phiSA[currAction][currAction*2 + 1] = timeFeature
        # QSA = np.array(np.random.rand(1, 6))
        stateVect.insert(step, [lenFeature, timeFeature, 0, 0])
        stateVect[step][3] = currAction
        stateVect[step][2] = reward

        # Control simulation
        if currAction > 2:
            phaseToChange = 2
        else:
            phaseToChange = 0
        # currTLS = traci.trafficlight.getCompleteRedYellowGreenDefinition("center")
        currTLS[0].phases[phaseToChange].duration = Actions[currAction]
        currTLS[0].phases[phaseToChange].minDur = Actions[currAction]
        currTLS[0].phases[phaseToChange].maxDur = Actions[currAction]
        traci.trafficlight.setCompleteRedYellowGreenDefinition("center", currTLS[0])

        # The Update
        if step > 0 and step % 75 == 0:
            episodeStates = [[]]
            A = 0
            R = 0
            for t in range(0, 74, 1):
                episodeStates.insert(t, stateVect[step-74+t])
            for t in range(0, 73, 1):
                S = episodeStates[t]
                R = S[2]
                A = S[3]
                Sprime = episodeStates[t+1]
                QSA = np.matmul(phiSA, theta)
                QdashSA = getQdash(phiSA, Sprime, theta)

                # print(Sprime)
                gradWqsa = [0]*13
                for i in range(13):
                    for j in range(6):
                        gradWqsa[i] = gradWqsa[i] + phiSA[j][i]
                # for i in range(6):
                #     print(phiSA[i])
                # print(gradWqsa)
                # break
                err = R + GAMMA * max(QdashSA) - QSA[A]
                const = ALPHA * err
                for i in range(13):
                    theta[i] = theta[i] + const * gradWqsa[i]
                # print(theta)

        # print(phiSA, sep='\n')
        # print(np.matrix(phiSA))
        # print('\n'.join(['\t'.join([str(cell) for cell in row]) for row in phiSA]))
        step += 1

        curWaittime = traci.edge.getWaitingTime('nc') + traci.edge.getWaitingTime('ec') + traci.edge.getWaitingTime('sc') + traci.edge.getWaitingTime('wc')
        totWaittimeSoFar = totWaittimeSoFar + curWaittime
        avgTime.append(totWaittimeSoFar/step)

        currVehsHere = traci.simulation.getArrivedNumber()
        totVehsSoFar = totVehsSoFar + currVehsHere
        numVehs.append(totVehsSoFar)
    traci.close()
    # print(phi)
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
    currTLS = traci.trafficlight.getCompleteRedYellowGreenDefinition("center")
    # if step == 0:
    #     print(currTLS, sep='\n')
    currTLS[0].phases[0].duration = dur1  # * 1000
    currTLS[0].phases[0].minDur = dur1  # * 1000
    currTLS[0].phases[0].maxDur = dur1  # * 1000

    currTLS[0].phases[2].duration = dur2  # * 1000
    currTLS[0].phases[2].minDur = dur2  # * 1000
    currTLS[0].phases[2].maxDur = dur2  # * 1000
    traci.trafficlight.setCompleteRedYellowGreenDefinition("center", currTLS[0])

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1
        curWaittime = traci.edge.getWaitingTime('nc') + traci.edge.getWaitingTime('ec') + traci.edge.getWaitingTime('sc') + traci.edge.getWaitingTime('wc')
        totWaittimeSoFar = totWaittimeSoFar + curWaittime
        avgTime.append(totWaittimeSoFar/step)

        currVehsHere = traci.simulation.getArrivedNumber()
        totVehsSoFar = totVehsSoFar + currVehsHere
        numVehs.append(totVehsSoFar)
        # if step == 1:
        #     print(currTLS, sep='\n')
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

    traci.start([sumoBinary, "-c", "data/sq.sumocfg"])
    step1, avgTime1, numVehs1 = runFixed(20, 20)

    traci.start([sumoBinary, "-c", "data/sq.sumocfg"])
    step2, avgTime2, numVehs2 = runFixed(25, 25)

    traci.start([sumoBinary, "-c", "data/sq.sumocfg"])
    step3, avgTime3, numVehs3 = runFixed(20, 25)

    traci.start([sumoBinary, "-c", "data/sq.sumocfg"])
    step4, avgTime4, numVehs4 = runFixed(25, 20)

    traci.start([sumoBinary, "-c", "data/sq.sumocfg"])
    step5, avgTime5, numVehs5 = runQFS()

    traci.start([sumoBinary, "-c", "data/sq.sumocfg"])
    step6, avgTime6, numVehs6 = runQFA()

    # formatter = FuncFormatter(seconds)
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
    plt.title('Traffic for a Square Junction: Average Waiting Time')
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
    plt.title('Traffic for a Square Junction: Number of Vehicles')
    plt.savefig('./Results/FixedVsQLFA/QFA_TNV.eps', bbox_inches='tight', format='eps', dpi=1200)
    plt.savefig('./Results/FixedVsQLFA/QFA_TNV.svg', bbox_inches='tight', format='svg', dpi=1200)
    interactive(False)
    plt.show()
