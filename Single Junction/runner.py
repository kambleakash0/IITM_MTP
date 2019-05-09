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
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
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
    cost = 0
    for i in range(4):
        cost = cost - state[i]
    return cost


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--gui", action="store_true", default=False, help="run the GUI version of sumo")
    options, args = optParser.parse_args()
    return options


def runMCC():
    stateVect = [[]]
    policyPi = []
    for i in range(0, 80, 1):
        policyPi.insert(i, random.randint(0, 5))
    QSA = np.random.rand(81, 6)
    Q30A = [[]]
    Q10A = [[]]
    Returns = [[0]*6 for x in range(81)]
    currTLS = []
    Actions = [20000, 25000, 30000, 20000, 25000, 30000]
    totTime = []
    avgTime = []
    numVehs = []
    avgNumVehs = []
    EPSILON = 0.20
    step = 0
    stateNorth = 0
    stateSouth = 0
    stateEast = 0
    stateWest = 0
    currAction = 0
    cost = 0
    totWaittimeSoFar = 0
    curWaittime = 0
    totVehsSoFar = 0
    currVehsHere = 0
    Q30A[0] = QSA[30]
    Q10A[0] = QSA[10]
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        stateNorth = traci.lane.getLastStepOccupancy('nc_0')
        stateSouth = traci.lane.getLastStepOccupancy('sc_0')
        stateEast = traci.lane.getLastStepOccupancy('ec_0')
        stateWest = traci.lane.getLastStepOccupancy('wc_0')
        tlsPhase = traci.trafficlight.getPhase("center")
        stateVect.insert(step, [getState(stateNorth), getState(stateEast), getState(stateSouth), getState(stateWest), getPhase(tlsPhase), 0, 0, 0])
        index = getStateIndex(stateVect[step])
        cost = getStateReturn(stateVect[step])
        currAction = policyPi[index]
        stateVect[step][5] = index
        stateVect[step][6] = currAction
        stateVect[step][7] = cost

        if currAction > 2:
            phaseToChange = 2
        else:
            phaseToChange = 0

        currTLS = traci.trafficlight.getCompleteRedYellowGreenDefinition("center")
        currTLS[0].phases[phaseToChange].duration = Actions[currAction]
        currTLS[0].phases[phaseToChange].duration1 = Actions[currAction]
        currTLS[0].phases[phaseToChange].duration2 = Actions[currAction]
        traci.trafficlight.setCompleteRedYellowGreenDefinition("center", currTLS[0])

        if step > 0 and step % 850 == 0:

            episodeStates = [[]]
            G_ep = 0
            index = 0
            currAction = 0
            cost = 0
            counter = [[0]*6 for i in range(81)]
            for t in range(0, 849, 1):
                episodeStates.insert(t, stateVect[step-t])
                currState = episodeStates[t]
                G_ep = G_ep + currState[7]
                currAction = currState[6]
                index = currState[5]

                for x in range(0, 80, 1):
                    for y in range(0, 5, 1):
                        if x == index and y == currAction:
                            Returns[x][y] += G_ep
                            counter[x][y] += 1
                            QSA[x][y] = Returns[x][y]/counter[x][y]
                            if random.uniform(0, 1) < EPSILON:
                                policyPi[index] = random.randint(0, 5)
                            else:
                                policyPi[index] = np.argmax(QSA[index])

        step += 1
        curWaittime = traci.edge.getWaitingTime('nc') + traci.edge.getWaitingTime('ec') + traci.edge.getWaitingTime('sc') + traci.edge.getWaitingTime('wc')
        totWaittimeSoFar = totWaittimeSoFar + curWaittime
        totTime.append(totWaittimeSoFar)
        avgTime.append(totWaittimeSoFar/step)

        currVehsHere = traci.simulation.getArrivedNumber()
        totVehsSoFar = totVehsSoFar + currVehsHere
        numVehs.append(totVehsSoFar)
        avgNumVehs.append(totVehsSoFar/(step))

        Q30A.append(QSA[30])
        Q10A.append(QSA[10])
    traci.close()
    sys.stdout.flush()
    return step, totTime, avgTime, numVehs, avgNumVehs


def runFixed(dur1, dur2):
    totTime = []
    avgTime = []
    numVehs = []
    avgNumVehs = []
    totTime.clear()
    avgTime.clear()
    numVehs.clear()
    avgNumVehs.clear()
    step = 0
    totWaittimeSoFar = 0
    totVehsSoFar = 0
    currTLS = traci.trafficlight.getCompleteRedYellowGreenDefinition("center")
    currTLS[0]._phases[0]._duration = dur1*1000
    currTLS[0]._phases[0]._duration1 = dur1*1000
    currTLS[0]._phases[0]._duration2 = dur1*1000

    currTLS[0]._phases[2]._duration = dur2*1000
    currTLS[0]._phases[2]._duration1 = dur2*1000
    currTLS[0]._phases[2]._duration2 = dur2*1000
    traci.trafficlight.setCompleteRedYellowGreenDefinition("center", currTLS[0])

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1
        curWaittime = traci.edge.getWaitingTime('nc') + traci.edge.getWaitingTime('ec') + traci.edge.getWaitingTime('sc') + traci.edge.getWaitingTime('wc')
        totWaittimeSoFar = totWaittimeSoFar + curWaittime
        totTime.append(totWaittimeSoFar)
        avgTime.append(totWaittimeSoFar/step)

        currVehsHere = traci.simulation.getArrivedNumber()
        totVehsSoFar = totVehsSoFar + currVehsHere
        numVehs.append(totVehsSoFar)
        avgNumVehs.append(totVehsSoFar/step)
    traci.close()
    sys.stdout.flush()
    return step, totTime, avgTime, numVehs, avgNumVehs


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

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    """traci.start([sumoBinary, "-c", "data/sq.sumocfg"])
    step1, totTime1, avgTime1, numVehs1, avgNumVehs1 = runFixed(20, 20)

    traci.start([sumoBinary, "-c", "data/sq.sumocfg"])
    step2, totTime2, avgTime2, numVehs2, avgNumVehs2 = runFixed(25, 25)

    traci.start([sumoBinary, "-c", "data/sq.sumocfg"])
    step3, totTime3, avgTime3, numVehs3, avgNumVehs3 = runFixed(20, 25)

    traci.start([sumoBinary, "-c", "data/sq.sumocfg"])
    step4, totTime4, avgTime4, numVehs4, avgNumVehs4 = runFixed(25, 20)"""

    traci.start([sumoBinary, "-c", "data/sq.sumocfg"])
    step4, totTime4, avgTime4, numVehs4, avgNumVehs4 = runMCC()

    formatter = FuncFormatter(seconds)
    # plt.figure()
    """plt.plot(totTime1, label='Fixed Duration: 15')
    plt.plot(totTime2, label='Fixed Duration: 20')
    plt.plot(totTime3, label='Fixed Duration: 25')"""
    fig, ax = plt.subplots()
    ax.yaxis.set_major_formatter(formatter)
    plt.plot(totTime4, label='Dynamic Duration: MCC')
    plt.legend(loc='center right')
    plt.xlabel('Simulation Steps')
    plt.ylabel('Total Waiting Time of all vehicles')
    plt.title('Traffic trend for a Square Junction: Total Waiting Time vs. Timesteps')
    plt.show()
