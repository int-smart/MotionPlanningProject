import numpy as np
import openravepy
import math
import sys
import random
import copy
# This file contains the planner for A*, SIPP and bidirectional RRT connect.
# from utils.config import *
from utils.config import *
random.seed(2)

def CheckCollisions(env, robot):
    if env.CheckCollision(robot) or robot.CheckSelfCollision():
        return True
    else:
        return False

class Node(object):
    def __init__(self,cfg,prt):
        assert (len(cfg) == NODE_SIZE), "Dimensions of node should be equal {}".format(NODE_SIZE)
        self._q = cfg
        self._parent = copy.deepcopy(prt)

    def __str__(self):
        print("The node config value and parent is ", self._q, self._parent)

    @property
    def config(self):
        return self._q

    @config.setter
    def config(self,cfg):
        self._q = cfg

    @property
    def parent(self):
        return self._parent

    @parent.setter
    def parent(self, prt):
        self._parent = copy.deepcopy(prt)

    def distanceMeasure(self, node, pr2):
        distance = 0.0
        first = node.config
        second = self._q
        sub = pr2.SubtractActiveDOFValues(first,second)
        for value in sub:
            distance = distance + value**2
        distance = distance**(0.5)
        return distance

def costOfPath(node, robot):
    cost = 0
    while node.parent is not None:
        cost = cost + node.distanceMeasure(node.parent, robot)
        node = node.parent
    return cost

class NodeTree(object):
    def __init__(self):
        self._tree = []

    def _addNode(self, node):
        self._tree.append(node)

    # TODO Can I change this to config comparision as that would be useful. I dont think this would be useful.
    def _deleteNode(self, node):
        if node in self._tree:
            self._tree.remove(node)

    def _getNode(self, index):
        return self._tree[index]

    def _generatePath(self, node):
        path = []
        while node.parent is not None:
            path.append(node.config)
            node = node.parent
        path.append(node.config)
        return path

    def _getNearestNeighbor(self, node, robot, delta):
        distance = delta
        minCost = sys.float_info.max
        # nearNode = Node([],None)
        for treeNode in self._tree:
            tempDistance = treeNode.distanceMeasure(node, robot)
            if tempDistance < distance:
                cost = costOfPath(treeNode, robot)
                if minCost>cost:
                    nearNode = treeNode
                    minCost = cost
        if nearNode is None:
            distance = sys.float_info.max
            for treeNode in self._tree:
                tempDistance = treeNode.distanceMeasure(node, robot)
                if distance > tempDistance:
                    distance = tempDistance
                    nearNode = treeNode
        return nearNode

    def _getNearestWitness(self, node, robot):
        # nearNode = Node([],None)
        minDist = sys.float_info.max
        nearNode = None
        for treeNode in self._tree:
            tempDistance = treeNode.distanceMeasure(node, robot)
            if minDist>tempDistance:
                nearNode = treeNode
                minDist = tempDistance
        return nearNode

    def _getTreeSize(self):
        return len(self._tree)

    def __str__(self, showTree):
        if showTree:
            print("The tree and tree size is ",self._getTreeSize(), self._tree)
        else:
            print("The tree size is ", self._getTreeSize())

class SST():
    def __init__(self, start, goal, maxIter, step, env, robot, smooth=False):
        self.startState = Node(start, None)
        self.goalState = Node(goal, None)
        self.maxIterations = maxIter
        self.stepSize = step
        self.env = env
        self.path = []
        self.robot = robot
        self.handles = []
        self.maxIterations = 500
        self.activeDistance = 0.5
        self.witnessDistance = 0.25
        self.lowerLimits, self.upperLimits = robot.GetActiveDOFLimits()
        self.lowerLimits[0] = -7.8
        self.lowerLimits[1] = -3.8
        self.upperLimits[0] = 7.8
        self.upperLimits[1] = 3.8
        self.lowerLimits[ORIENTATION] = -math.pi
        self.upperLimits[ORIENTATION] = math.pi
        if self.startState.config[ORIENTATION] < self.lowerLimits[ORIENTATION]:
            self.startState.config[ORIENTATION] = self.lowerLimits[ORIENTATION]
        if self.startState.config[ORIENTATION] > self.upperLimits[ORIENTATION]:
            self.startState.config[ORIENTATION] = self.upperLimits[ORIENTATION]
        if self.goalState.config[ORIENTATION] < self.lowerLimits[ORIENTATION]:
            self.goalState.config[ORIENTATION] = self.lowerLimits[ORIENTATION]
        if self.startState.config[ORIENTATION] > self.upperLimits[ORIENTATION]:
            self.startState.config[ORIENTATION] = self.upperLimits[ORIENTATION]
        self.tree_forward = NodeTree()
        self.tree_backward = NodeTree()
        self.doSmooth = smooth

    def isGoal(self, node):
        if node.distanceMeasure(self.goalState, self.robot)<self.stepSize:
            return True
        else:
            return False

    def getPath(self):
        return self.path

    def buildSST(self):
        i = 0
        activeSet = NodeTree()
        inactiveSet = NodeTree()
        activeSet._addNode(self.startState)
        witnessSet = NodeTree()

        witnessNode = self.startState
        witnessNode.parent = self.startState
        witnessSet._addNode(self.startState)
#TODO I am currently forward propagating the nearest node to the random node
#by step size so the activeDistance used to search in the activeSet should be
#greater than step.
        while i<=self.maxIterations:
            sampleNode = self.randomConfig()
            nearestActiveNode = activeSet._getNearestNeighbor(sampleNode, self.robot, self.activeDistance)
            newNode = self.newConfig(nearestActiveNode, sampleNode)

            if newNode is not None:
                nearestWitness = witnessSet._getNearestWitness(newNode,self.robot)
                if nearestWitness.distanceMeasure(newNode, self.robot)>self.witnessDistance:
                    nearestWitness = newNode
                    nearestWitness.parent = None
                    witnessSet._addNode(nearestWitness)
                peerNode = nearestWitness.parent
                if peerNode is None or costOfPath(newNode, self.robot)<costOfPath(peerNode, self.robot):
                    activeSet._deleteNode(peerNode)
                    inactiveSet._addNode(peerNode)

                    nearestWitness.parent = newNode
                    activeSet._addNode(newNode)

                    while True:
                        nodeList = []
                        deleteFlag = False
                        for node in inactiveSet._tree:
                            nodeList.append(node.parent)
                        for node in activeSet._tree:
                            nodeList.append(node.parent)
                        for node in inactiveSet._tree:
                            if node not in nodeList:
                                inactiveSet._deleteNode(node)
                                deleteFlag = True

                        if deleteFlag == False:
                            break
            i = i+1
        return

    def randomConfig(self):
        randConfig = []
        for index in range(len(self.goalState.config)):
            rnge = self.upperLimits[index] - self.lowerLimits[index]
            val = self.lowerLimits[index] + (rnge*random.random())
            randConfig.append(val)
        temp = Node(randConfig, None)
        return temp


    def IsSafePoint(self,config):
        for index in range(len(config)):
            if config[index] < self.lowerLimits[index] or config[index] > self.upperLimits[index]:
                return False
        return True


    def newConfig(self, nearState, randState):
        nearConfig = nearState.config
        randConfig = randState.config
        newConfig = []
        distance = nearState.distanceMeasure(randState, self.robot)
        sub = self.robot.SubtractActiveDOFValues(randConfig,nearConfig)
        for index in range(len(sub)):
            temp = nearConfig[index] + ((self.stepSize*sub[index])/distance)
            newConfig.append(temp)
        if self.IsSafePoint(newConfig):
            self.robot.SetActiveDOFValues(newConfig)
            if CheckCollisions(self.env, self.robot):
                return None
            else:
                newState = Node(newConfig, nearState)
                return newState
        else:
            return None



if __name__ == "__main__":
    a = Node([], None)
    b = Node([x for x in range(3)], a)
    c = Node([x+1 for x in range(4)],b)
    tr = NodeTree()
    tr._addNode(a)
    tr._addNode(b)
    tr._addNode(c)
    tr._deleteNode(b)
    te = 1
