import numpy as np
import openravepy
import math
import sys
import random
import copy
from utils.config import *

try:
    import Queue as q
    import heapq
except ImportError:
    import queue as q

# TODO: First the hValue should be calculated once so the startConfig and goalConfig
# should be stored with the state. This is not done in the case now as
# we are having SIPP(None, None) which initializes State which calculates the
# hValue which is not possible now as startConfig would be None and something just
# think about this. Similarly the cost of moving one step edgeCost is one now
# but it should be the euclidean distance. So change it.
# The notion of a well-formed infrastructure easily extends to 3D space so that a
# path is guaranteed as long as each flying car starts at a parking
# node,  has  an  empty  parking  node  as  its  endpoint,
# and does not travel through other parking nodes on its way
# to its goal.
# Parking nodes are therefore introduced so that you dont have your goal
# right in the middle of the road as that would block others path.
class State(object):
    goalConfig = []
    startConfig = []
    def __init__(self, config, parent):
        """
        In original implementation they have stored the time intervals as
        the safe one rather than the one that I am storing which is
        the opposite of that.
        :param config:
        :param parent:
        """
        self._node = config
        self._tInterval = []
        self._parent = parent
        self._cValue = 0    #cValue is the gValue of normal A*
        self._hValue = SIPP.euclideanMetric(config, State.goalConfig)
        self._eValue = 0    #eValue is the fValue is normal A*
        self._arrTime = 0   #This is the arrival time
        if self.parent is not None:
            self._edgeCost = SIPP.euclideanMetric(config, parent.node)
        else:
            self._edgeCost = 0

    @staticmethod
    def setParameters(srtConfig, glConfig):
        State.startConfig = srtConfig
        State.goalConfig = glConfig

    @property
    def edgeCost(self):
        return SIPP.euclideanMetric(self.node, self.parent.node)

    @edgeCost.setter
    def edgeCost(self, value):
        self._edgeCost = value

    @property
    def cValue(self):
        return self._cValue

    @cValue.setter
    def cValue(self, value):
        self._cValue = value

    @property
    def hValue(self):
        return self._hValue

    @hValue.setter
    def hValue(self, value):
        self._hValue = value

    @property
    def eValue(self):
        return self._eValue

    @eValue.setter
    def eValue(self, value):
        self._eValue = value

    @property
    def parent(self):
        return self._parent

    @parent.setter
    def parent(self, parentNode):
        self._parent = parentNode
        # self._edgeCost = SIPP.euclideanMetric(config, parent.node)

    @property
    def node(self):
        return self._node

    @node.setter
    def node(self, nodeConfig):
        self._node = nodeConfig

    def addtValue(self, value):
        self._tInterval.append(value)

    def deletetValue(self, value):
        if value in self._tInterval:
            self._tInterval.remove(self._tInterval.index(value))

    def searchtValue(self, value):
        if value in self._tInterval:
            return True
        else:
            return False

    def gettValue(self):
        return self._tInterval

    @property
    def arrTime(self):
        return self._arrTime

    @arrTime.setter
    def arrTime(self, value):
        self._arrTime = value
# The arrival time is the time when the first time it is occupied


class SIPP(object):
    TInterval = dict()
    def __init__(self, startConfig, goalConfig, env, robot, stepX=0.25, stepY=0.15, stepTheta=0.25, tolerance=0.6, hr = "euclidean", connection=8):
        State.setParameters(startConfig, goalConfig)
        self.startState = State(startConfig,None)
        self.goalState = State(goalConfig,None)
        self.stepSizeX = stepX
        self.stepSizeY = stepY
        self.stepSizeTheta = stepTheta
        self.tolValue = tolerance
        self.heuristic = hr
        self.connectionType = connection
        self.handler = []
        self.clock = STARTING_TIME
        self.closedSet = set()
        self.path = []
        self.env = env
        self.robot = robot
        self.startState = State(startConfig, None)
        self.goalState = State(goalConfig, None)

    @staticmethod
    def CheckCollisions(env, robot, config):
        robot.SetActiveDOFValues(config)
        if env.CheckCollision(robot) or robot.CheckSelfCollision():
            return True
        else:
            return False

    @staticmethod
    def getEightConnectedNeighbors(currentState, env, robot, stepSizeX, stepSizeY, stepSizeTheta):
        temp = []
        nextCells = []
        state = currentState.node
        # This should give us the same states for all robots. So a robot that
        # has visited some state the next robot should get that same datastructure
        # when it finds its successors
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                for k in [-1, 0, 1]:
                    if i == j == k == 0:
                        continue
                    if (state[2] + k * stepSizeTheta) >= math.pi:
                        Z = state[2] - 2 * math.pi
                    elif (state[2] + k * stepSizeTheta) <= -1 * math.pi:
                        Z = state[2] + 2 * math.pi
                    else:
                        Z = state[2]
                    temp = [state[0] + i * stepSizeX, state[1] + j * stepSizeY, Z + k * stepSizeTheta]
                    if SIPP.CheckCollisions(env,robot,temp):
                        continue
                    else:
                        tempState = State(temp, currentState)
                        nextCells.append(tempState)
        return nextCells

    @staticmethod
    def euclideanMetric(current, goal):
        temp = [current[i] - goal[i] for i in range(len(current))]
        temp = np.asarray(temp)
        heuristic = np.linalg.norm(temp, 2)
        return heuristic

    @staticmethod
    def addEntryToQueue(container, priority, entry):
        newEntry = copy.deepcopy(entry)
        heapq.heappush(container,([priority, newEntry]))
        heapq.heapify(container)

    @staticmethod
    def getEntryFromQueue(container):
        return heapq.heappop(container)

    def getTValue(self, state):
        if self.TInterval.has_key(tuple(state.node)):
            return self.TInterval[tuple(state.node)]
        else:
            return []

    def generatePath(self):
        path = []
        state = copy.deepcopy(self.goalState)
        while state.parent is not None:
            path.append(state.node)
            if SIPP.TInterval.has_key(tuple(state.node)):
                SIPP.TInterval[tuple(state.node)].append(state.arrTime)
            else:
                SIPP.TInterval[tuple(state.node)] = []
                SIPP.TInterval[tuple(state.node)].append(state.arrTime)
            state = state.parent
        path.append(state.node)

        return path

    def getSuccessors(self, state, timeStep):
        """
        This method is trying to get the neighbors, then using the safe interval list
        of all the neighbors it checks if the start and end time of our movement in and out
        of this node falls in the safe interval. If it does, t_arr is assigned the earliest
        arrival time in L(list of safe intervals), cost is calculated and
        :param state:
        :param timeStep:
        """
        successors = []
        for indState in self.getEightConnectedNeighbors(state, self.env, self.robot, self.stepSizeX, self.stepSizeY, self.stepSizeTheta):
            L = self.getTValue(indState)                    #Getting the safe interval list
            dt = EDGETIMECOST                           #time to traverse to this successor node from current node = 1
            counter = timeStep                          #Here I am starting from timeStep as timeStep has already passed
            start_time = 0
            end_time = float("inf")
            currentInterval = self.getTValue(state)
            current_time = timeStep

            copyInterval = copy.deepcopy(currentInterval)
            copyInterval.append(current_time)
            copyInterval.sort()
            if copyInterval.index(current_time) == len(copyInterval)-1:
                current_end_time = float("inf")
            else:
                current_end_time = copyInterval[copyInterval.index(current_time)+1]-dt
            # while current_time not in currentInterval:
            #     current_time = current_time+1
            # current_end_time = current_time-1
            #The first case below is when the list of intervals is empty
            if len(L) == 0:
                start_time = 0
                end_time = float("inf")
                if end_time > timeStep + dt and start_time<current_end_time:            #TODO Check Condition Here None action would be problem
                    # I have added timestep+dt in startTime because the when I reach there
                    t_arr = timeStep + dt
                    cost = indState.edgeCost
                    #TODO Add Check condition if the node is too near the other robots goal, which can be problem. Not right now though
                    # indState.addtValue(t_arr)
                    indState.arrTime = t_arr
                    successors.append(indState)

            # The second case below is when the list of intervals is not empty
            else:
                for interval in range(len(L)+1):
                    if interval == 0:
                        if L[interval] > 0:
                            end_time = L[interval]
                            start_time = 0
                    elif interval == len(L):
                        end_time = float("inf")
                        start_time = L[interval-1] + dt
                    else:
                        end_time = L[interval]
                        start_time = L[interval-1] + dt
                    if end_time > timeStep + dt and start_time <current_end_time:  # TODO Check Condition Here None action would be problem
                        t_arr = timeStep + dt
                        cost = indState.edgeCost
                        # TODO Add Check condition if the node is too near the other robots goal, which can be problem. Not right now though
                        # indState.addtValue(t_arr)
                        indState.arrTime = t_arr
                        successors.append(indState)
        return successors

    def runSIPP(self):
        # TODO Change the start state values here and choose if you want
        # TODO to put all the robots in SIPP or declare SIPP for each one
        openPriorityQueue = []
        closedList = set()
        nodesList = []
        CValue = {}
        TValue = {}

        self.startState.cValue = 0
        self.startState.eValue = self.euclideanMetric(self.startState.node, self.goalState.node)
        self.startState.arrTime = self.clock
        SIPP.addEntryToQueue(openPriorityQueue, self.startState.eValue, self.startState)
        CValue[tuple(self.startState.node)] = 0
        TValue[tuple(self.startState.node)] = self.clock
        currentState = self.startState
# TODO the openPriorityQueue can not be empty as it owuld take infinite time so rather cap it with max iterations
        while self.euclideanMetric(currentState.node, self.goalState.node)>1.0:
            currentState = SIPP.getEntryFromQueue(openPriorityQueue)[1]
            # TODO Here the
            # datastructure that would come would have state and priority so
            # change this statement to index the state. RIght now I am not
            # doing it to understand the algo first
            successorList = self.getSuccessors(currentState, currentState.arrTime)
            for successor in successorList:
                flag = 0                    #Assuming the node is not present in the closedList
                for element in closedList:
                    if element == tuple(successor.node):
                        flag = 1            #If the node is found in closedList make flag = 1
                    else:
                        flag = 0            #Else make the flag = 0

                if flag == 0:
                    closedList.add(tuple(successor.node))
                    CValue[tuple(successor.node)] = float("inf")
                    TValue[tuple(successor.node)] = float("inf")
                if CValue[tuple(successor.node)] > currentState.cValue+successor.edgeCost or TValue[tuple(successor.node)] > successor.arrTime:
                    CValue[tuple(successor.node)] = min(currentState.cValue+successor.edgeCost, CValue[tuple(successor.node)])
                    TValue[tuple(successor.node)] = min(TValue[tuple(successor.node)], successor.arrTime)
                    successor.cValue = currentState.cValue+successor.edgeCost                       #Check if this is already done in the
                    successor.eValue = currentState.cValue+successor.edgeCost+successor.hValue      #getSuccessor function. Do only once.
                    # successor.arrTime = currentState.arrTime + 1

                    SIPP.addEntryToQueue(openPriorityQueue, successor.eValue, successor)
                    self.handler.append(self.env.plot3(np.array([successor.node[0], successor.node[1], 0.1]), 5.0, colors=np.array(((1, 0, 0)))))
                    for entry in openPriorityQueue:
                        term = (entry[1].cValue > CValue[tuple(successor.node)] and entry[1].arrTime > TValue[tuple(successor.node)]) or (entry[1].cValue >= CValue[tuple(successor.node)] and entry[1].arrTime > TValue[tuple(successor.node)]) or (entry[1].cValue > CValue[tuple(successor.node)] and entry[1].arrTime >= TValue[tuple(successor.node)])
                        if entry[1].node == successor.node and term:
                            openPriorityQueue.remove(entry)
                    heapq.heapify(openPriorityQueue)
        self.goalState.parent = currentState
        self.goalState.arrTime = currentState.arrTime+1
        self.path = self.generatePath()
        self.path.reverse()


    def setStartState(self, startConfig):
        self.startState = State(startConfig, None)

    def setGoalState(self, goalConfig):
        self.goalState = State(goalConfig, None)

if __name__=="__main__":
    priortyList = [1, 4, 2, 3]
    # State.setParameters(startValue[priortyList[0]], goalValue[priortyList[0]])
    # solverSIPP = SIPP(None, None)
    # for index in range(len(priortyList)):
    #     State.setParameters(startValue[priortyList[index]], goalValue[priortyList[index]])
    #     solverSIPP.setStartState(startValue[priortyList[index]])
    #     solverSIPP.setGoalState(goalValue[priortyList[index]])
    #     solverSIPP.runSIPP()
