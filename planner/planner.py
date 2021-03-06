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
# TODO Add the collision checking when connecting the node to the random node in the connect
# algorithm not in randConfig as randConfig takes much more time

#TODO Try and see if removing the other PR2 other than ambulance to see if collisionchecker is checking them and therefore not giving answers
#TODO Why is goal again and again coming
#TODO Problem: randState is coming out startState and nearState is coming out goal
#TODO The backWard tree has the same node added multiple times
#TODO randState in extend is coming out same as startState
#TODO Advanced in connect or extend is maybe not changing the nearState and thats why just adding one nearState to tree
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

    def _getNearestNeighbor(self, node, robot):
        distance = sys.float_info.max
        # nearNode = Node([],None)
        for treeNode in self._tree:
            tempDistance = treeNode.distanceMeasure(node, robot)
            if distance > tempDistance:
                distance = tempDistance
                nearNode = treeNode
        return nearNode

    def _getTreeSize(self):
        return len(self._tree)

    def __str__(self, showTree):
        if showTree:
            print("The tree and tree size is ",self._getTreeSize(), self._tree)
        else:
            print("The tree size is ", self._getTreeSize())

def Astar():
    raise NotImplementedError

class BiRRTConnect():
    def __init__(self, start, goal, maxIter, step, env, robot, smooth=False):
        self.startState = Node(start, None)
        self.goalState = Node(goal, None)
        self.maxIterations = maxIter
        self.stepSize = step
        self.env = env
        self.path = []
        self.robot = robot
        self.handles = []
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
        # TODO I would suggest constraining the x, y and orientation of the PR2 according to the walls before
        # TODO implementing the solution
        # self.lowerLimits =
        # self.upperLimits =
    def isGoal(self, node):
        if node.distanceMeasure(self.goalState, self.robot)<self.stepSize:
            return True
        else:
            return False

    def getPath(self):
        return self.path

    def buildBiRRTConnect(self):
        direction = FORWARD
        self.tree_forward._addNode(copy.deepcopy(self.startState))
        self.tree_backward._addNode(copy.deepcopy(self.goalState))
        finalForwardPath = []
        finalBackwardPath = []
        finalPath = []
        doneFlag = False
        for index in range(self.maxIterations):
            qRand = self.randomConfig()
            if direction == FORWARD:
                nearState = copy.deepcopy(self.tree_forward._getNearestNeighbor(qRand, self.robot))
                statusExtend = self.extend(self.tree_forward,qRand, nearState)
                if statusExtend == "Reached":
                    aim = copy.deepcopy(qRand)
                elif statusExtend == "Advanced":
                    aim = copy.deepcopy(nearState)
                else:
                    continue
                if self.connect(self.tree_backward,aim) == "Reached":
                    doneFlag = True
            elif direction == BACKWARD:
                nearState = copy.deepcopy(self.tree_backward._getNearestNeighbor(qRand, self.robot))
                statusExtend = self.extend(self.tree_backward, qRand, nearState)
                if statusExtend == "Reached":
                    aim = copy.deepcopy(qRand)
                elif statusExtend == "Advanced":
                    aim = copy.deepcopy(nearState)
                else:
                    continue
                if self.connect(self.tree_forward,aim) == "Reached":
                    doneFlag = True
            direction = -1 * direction
            if doneFlag == True:
                finalForwardPath = self.tree_forward._generatePath(self.tree_forward._getNode(self.tree_forward._getTreeSize()-1))
                finalBackwardPath = self.tree_backward._generatePath(self.tree_backward._getNode(self.tree_backward._getTreeSize()-1))
                finalForwardPath.reverse()
                finalForwardPath = finalForwardPath[:-1]
                finalForwardPath = finalForwardPath+finalBackwardPath
                print("Reached the goal")
                if self.doSmooth:
                    self.path = self.smoothenPath(finalForwardPath)
                else:
                    self.path = finalForwardPath
                for index in range(len(self.path)):
                    self.handles.append(self.env.plot3(np.array(self.path[index]), 2.0, colors=np.array(((1,0,0)))))
                return "success"
        print("Fail to find solution")
        return "Failure"

    def randomConfig(self):
        randConfig = []
        for index in range(len(self.goalState.config)):
            rnge = self.upperLimits[index] - self.lowerLimits[index]
            val = self.lowerLimits[index] + (rnge*random.random())
            randConfig.append(val)
        temp = Node(randConfig, None)
        return temp

    def connect(self, tree, randState):
        status = "Advanced"
        nearState = copy.deepcopy(tree._getNearestNeighbor(randState, self.robot))
        while status == "Advanced":
            status = self.extend(tree, randState, nearState)
        return status

    def extend(self, tree, randState, nearState):
        newState = self.newConfig(nearState, randState)
        status = ""
        if newState is None:
            status = "Trapped"
            return status
        else:
            if randState.distanceMeasure(newState, self.robot)<self.stepSize:
                tree._addNode(copy.deepcopy(newState))
                randState.parent = newState
                self.robot.SetActiveDOFValues(randState.config)
                if not CheckCollisions(self.env, self.robot):
                    tree._addNode(copy.deepcopy(randState))
                status = "Reached"
                return status
            else:
                tree._addNode(copy.deepcopy(newState))
                self.handles.append(self.env.plot3(np.array([newState.config[0], newState.config[1], 0.1]), 5.0, colors=np.array(((1, 0, 0)))))
                status = "Advanced"
                #TODO this nearstate is notchanging in the above connect function on line 211. In the next line nearState is being declared again
                nearState.config = newState.config
                nearState.parent = newState.parent
                return status

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

    def smoothenPath(self, generatedPath):
        path = copy.deepcopy(generatedPath)
        for iteration in range(200):
            firstIndex = 0
            secondIndex = 0
            while firstIndex >= secondIndex:
                firstIndex = random.randint(0, len(path)-1)
                secondIndex = random.randint(0, len(path)-1)
            firstNode = Node(path[firstIndex], None)
            secondNode = Node(path[secondIndex], None)
            if firstNode.distanceMeasure(secondNode, self.robot)<self.stepSize:
                path = path[:firstIndex+1] + path[secondIndex:]
            else:
                success = True
                while firstNode.distanceMeasure(secondNode, self.robot)>=self.stepSize:
                    newConfig = []
                    tempConfig = []
                    distance = firstNode.distanceMeasure(secondNode, self.robot)
                    tempConfig = secondNode.config
                    sub = self.robot.SubtractActiveDOFValues(tempConfig, firstNode.config)
                    for index in range(len(sub)):
                        temp = firstNode.config[index] + (self.stepSize*(tempConfig[index])/distance)
                        newConfig.append(temp)
                    self.robot.SetActiveDOFValues(newConfig)
                    if CheckCollisions(self.env, self.robot):
                        success = False
                        break
                    else:
                        firstNode.config = newConfig
                if success:
                    path = path[:firstIndex + 1] + path[secondIndex:]
        return path


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
