import numpy
import planner as backward
import SIPP
import openravepy

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

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

def priorityPlanner(env, robot, start, goal):
    maxIter = 1000
    step = 0.5
    priority = dict()
    count = 0
    robots = env.GetRobots()
    closedList = []             #vehicles
    openList = []              #obstacles
    agent = Robot(start, goal, robot)
    tempList = []
    tempList.append(agent)
    openList.append(tempList)
    doneFlag = False
    while len(openList)!=0:
        agentList = openList.pop()
        for agent in agentList:
            planFor = agent.getRobot()
            if planFor not in closedList:
                closedList.append(planFor)
                priority[planFor] = count
                count = count+1
                obstacleL, status = planBackward(env, planFor, robots, closedList, agent.getStart(), agent.getGoal(), step, maxIter)
                openList.append(obstacleL)
    if doneFlag == False:
        print('Fail to reach to a solution')

#TODO 1. the below function should generate random goals
#TODO 2. The canPlan is false right now. Maybe add a condition if canPlan fails for 5 times maybe the ordering is wrong and that would call for order
def planBackward(env, planFor, robots, closedList, start, goal, step, maxIter):
    for vehicle in robots:
        env.RemoveKinBody(vehicle)
    env.AddRobot(planFor)
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

    obstacleList = []
    finalList = []
    for index in range(len(path)):
        planFor.SetActiveDOFValues(path[index])
        for vehicle in robots:
            if env.CheckCollision(planFor, vehicle) == True and vehicle not in closedList and vehicle is not planFor:
                obstacleList.append(vehicle)
    obstacleList = list(set(obstacleList))
    for obs in obstacleList:
        goal = backwardPlanner.randomConfig().config
        start = obs.GetActiveDOFValues()
        tempAgent = obs
        temp = Robot(start, goal, tempAgent)
        finalList.append(temp)

    return finalList, True