#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
from planner import priorityPlanner as prp
from planner import SIPP as multiPlanner
import random
# TODO: The goal and start state of the agent on lines 60, 61 is coming same. Fix it
# TODO: Currently the configurations are not colliding in the SIPP as all config can be different even if
# the orientation is different by 0.00001. SO add the distance measure so that the path for one robot at time
# t should not include any node at that time t for other robots in an circular region of distance d.
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def removeAllRobots(env):
    for robot in env.GetRobots():
        env.RemoveKinBody(robot)

def addIndividualRobot(env, planFor, startConfigurations):
    env.AddRobot(planFor)
    planFor.SetActiveDOFValues(startConfigurations[planFor])

def removeIndividualRobot(env, planFor):
    env.RemoveKinBody(planFor)

def addAllRobots(env, startConfigurations):
    for robot in startConfigurations.keys():
        env.AddRobot(robot)
        robot.SetActiveDOFValues(startConfigurations[robot])

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)


    env.Reset()
    # load a scene from ProjectRoom environment XML file
    # env.Load('./data/pr2test2.env.xml')
    env.Load('./data/4PR2_door.xml')
    time.sleep(0.1)

    startConfigurations = dict()
    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    for i in range(len(env.GetRobots())):
         robot = env.GetRobots()[i]
         tuckarms(env, robot)
         robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis, [0, 0, 1])
         startConfigurations[robot] = list(robot.GetActiveDOFValues())
         # tuck in the PR2's arms for driving


    # Robot[0] is the ambulance and all others are vehicles
    ambulance = env.GetRobots()[0]
    ambulanceStartConfig = list(startConfigurations[ambulance])
    ambulanceGoalConfig = [4.60594, 0.28479, 0]
    TI = dict()
    # ambulanceGoalConfig = [2.6,-1.3,-pi/2]

    random.seed(2)
    #### YOUR CODE HERE ####
    priorList = prp.priorityPlanner(env, startConfigurations, ambulance, ambulanceStartConfig, ambulanceGoalConfig)
    for i in range(len(priorList.values())):
        li = list(reversed(sorted(priorList.values())))
        prior = li[i]
        agent = priorList.keys()[priorList.values().index(prior)]
        # Clearing the environment of all robots so as to plan for individual robots according to priorty
        removeAllRobots(env)
        ambulanceStartConfig = agent.getStart()
        ambulanceGoalConfig = agent.getGoal()
        ambulance = agent.getRobot()
        addIndividualRobot(env, ambulance, startConfigurations)
        multiPlan = multiPlanner.SIPP(ambulanceStartConfig, ambulanceGoalConfig, env, ambulance)
        multiPlan.TInterval = TI
        multiPlan.runSIPP()
        TI = multiPlanner.SIPP.TInterval
        removeIndividualRobot(env, ambulance)
        addAllRobots(env, startConfigurations)

    # ambulance = env.GetRobots()[1]
    # ambulanceStartConfig = list(ambulance.GetActiveDOFValues())
    # ambulanceGoalConfig = [0, 0, 0]
    # multiPlan1 = multiPlanner.SIPP(ambulanceStartConfig, ambulanceGoalConfig, env, ambulance)
    # multiPlan1.TInterval = multiPlan.TInterval
    # multiPlan1.runSIPP()
    print("I am done")


    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")
    env.Destroy()
