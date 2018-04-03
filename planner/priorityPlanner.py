import numpy

from collections import defaultdict

import copy
import planner as backward
from utils.CyclicGraph import CyclicGraph
import SIPP
import openravepy

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def CheckCycle(prDict, env, vertexIndex):
    cg = CyclicGraph(len(env.GetRobots()))
    for keys in prDict.keys():
        for obstacle in prDict[keys]:
            cg.addEdge(vertexIndex[keys.getRobot()], vertexIndex[obstacle.getRobot()])
    priority = CyclicGraph.tSort(cg)
    if cg.isCyclic() == 1:
        return True, None
    else:
        return False, priority


class Robot(object):
    def __init__(self, start, goal, robot):
        self.startState = start
        self.goalState = goal
        self.robot = robot

    def getRobot(self):
        return self.robot

    def getStart(self):
        return self.startState

    def getGoal(self):
        return self.goalState

def priorityPlanner(env, initialStart, robot, start, goal):
    maxIter = 1000
    step = 0.5
    vertexIndex = dict()
    count = 0
    robots = env.GetRobots()
    closedList = []             #vehicles
    priorityDict = defaultdict(list)
    openList = []              #obstacles
    agent = Robot(start, goal, robot)
    tempList = []
    tempList.append(agent)
    openList.append(tempList)

    for rob in range(len(env.GetRobots())):
        vertexIndex[env.GetRobots()[rob]] = rob

    doneFlag = False
    while doneFlag==False:
        while len(openList)!=0:
            agentList = openList.pop()
            for agent in agentList:
                planFor = agent.getRobot()
                if planFor not in closedList:
                    closedList.append(planFor)
                    count = count+1
                    obstacleL, status = planBackward(env, planFor, robots, initialStart, closedList, agent.getStart(), agent.getGoal(), step, maxIter)
                    openList.append(obstacleL)
                    priorityDict[agent] = obstacleL
        flag, priorty = CheckCycle(priorityDict, env, vertexIndex)
        doneFlag = not flag
        # priorty.reverse()
        finalPriorty = dict()
    for index in range(len(priorty)):
        allAgentList = priorityDict.keys()
        for indAgent in allAgentList:
            if indAgent.getRobot() == vertexIndex.keys()[vertexIndex.values().index(priorty[index])]:
                finalPriorty[indAgent] = index

    if doneFlag == False:
        print('Fail to reach to a solution')
    return finalPriorty

#TODO 1. the status in the below function is always true change it to consider cases when this order is not correct
#TODO 2. The canPlan is false right now. Maybe add a condition if canPlan fails for 5 times maybe the ordering is wrong and that would call for order
def planBackward(env, planFor, robots, initialStart,closedList, start, goal, step, maxIter):
    for i in range(len(robots)):
        env.RemoveKinBody(robots[i])
    env.AddRobot(planFor)
    planFor.SetActiveDOFValues(initialStart[planFor])
    backwardPlanner = backward.BiRRTConnect(start, goal, maxIter, step, env, planFor, False)
    canPlan = False  # Currently we can not plan
    while canPlan == False:  # While we do not have any plan for the backward route for the ambulance. We try to find one
        status = backwardPlanner.buildBiRRTConnect()
        if status == "success":
            canPlan = True
    path = backwardPlanner.getPath()
    env.RemoveKinBody(planFor)
    for vehicle in robots:
        env.AddRobot(vehicle)
        vehicle.SetActiveDOFValues(initialStart[vehicle])

    obstacleList = []
    finalList = []
    for index in range(len(path)):
        planFor.SetActiveDOFValues(path[index])
        for vehicle in robots:
            if env.CheckCollision(planFor, vehicle) == True and vehicle not in closedList and vehicle is not planFor:
                obstacleList.append(vehicle)
    planFor.SetActiveDOFValues(initialStart[planFor])
    obstacleList = list(set(obstacleList))
    for obs in obstacleList:
        newgoal = backwardPlanner.randomConfig().config
        obs.SetActiveDOFValues(newgoal)
        while env.CheckCollision(obs) == True:
            newgoal = backwardPlanner.randomConfig().config
            obs.SetActiveDOFValues(newgoal)
        obs.SetActiveDOFValues(initialStart[obs])
        newstart = initialStart[obs]
        tempAgent = obs
        temp = Robot(newstart, newgoal, tempAgent)
        finalList.append(temp)

    return finalList, True