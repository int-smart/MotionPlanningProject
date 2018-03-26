import numpy
import planner as backward
import SIPP
import openravepy

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def priorityPlanner(env, robot, start, goal):
    maxIter = 1000
    step = 0.5
    priority = dict()
    count = 0
    robots = env.GetRobots()
    closedList = []             #vehicles
    openList = []              #obstacles
    openList.append(list(robot))
    doneFlag = False
    while len(openList)!=0:
        planList = openList.pop()
        for planFor in planList:
            if planFor not in closedList:
                closedList.append(planFor)
                priority[planFor] = count
                count = count+1
                planBackward(env, planFor, robots, closedList)
    if doneFlag == False:
        print('Fail to reach to a solution')

#TODO 1. the below function should generate random goals
#TODO 2. The canPlan is false right now. Maybe add a condition if canPlan fails for 5 times maybe the ordering is wrong and that would call for order
def planBackward(env, planFor, robots, closedList):
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
    for index in range(len(path)):
        planFor.SetActiveDOFValues(path[index])
        for vehicle in vehicles:
            if env.CheckCollision(planFor, vehicle) == True:
                obstacleList.append(vehicle)
    obstacles.append(obstacleList)
    for obs in obstacleList: