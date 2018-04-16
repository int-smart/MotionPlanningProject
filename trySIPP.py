#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
from planner import priorityPlanner as prp
from planner import SIPP as multiPlanner
import random
import numpy as np
# TODO: The goal and start state of the agent on lines 60, 61 is coming same. Fix it
# TODO: Currently the configurations are not colliding in the SIPP as all config can be different even if
# the orientation is different by 0.00001. SO add the distance measure so that the path for one robot at time
# t should not include any node at that time t for other robots in an circular region of distance d.
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

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

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def ConvertPathToTrajectory(robot,path=[]):
#Path should be of the form path = [q_1, q_2, q_3,...], where q_i = [x_i, y_i, theta_i]
    if not path:
	    return None
    # Initialize trajectory
    traj = RaveCreateTrajectory(env,'')
    traj.Init(robot.GetActiveConfigurationSpecification())
    for i in range(0,len(path)):
	    traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeAffineTrajectory(traj,maxvelocities=ones(3),maxaccelerations=5*ones(3))
    return traj

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
    env.Load('./data/pr2test3.env.xml')
    # env.Load('./data/4PR2_door.xml')
    time.sleep(0.1)

    startConfigurations = dict()
    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    for i in range(len(env.GetRobots())):
         robot = env.GetRobots()[i]
         tuckarms(env, robot)

         # tuck in the PR2's arms for driving

    with env:
        for i in range(len(env.GetRobots())):
            robot = env.GetRobots()[i]
            robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis, [0, 0, 1])
            startConfigurations[robot] = list(robot.GetActiveDOFValues())
        # Robot[0] is the ambulance and all others are vehicles
        robotList = env.GetRobots()
        TI = dict()
        # ambulanceGoalConfig = [2.6,-1.3,-pi/2]
        handler = []
        random.seed(2)
        #### YOUR CODE HERE ####
        # priorList = prp.priorityPlanner(env, startConfigurations, ambulance, ambulanceStartConfig, ambulanceGoalConfig)
        # for i in range(len(priorList.values())):
        #     li = list(reversed(sorted(priorList.values())))
        #     prior = li[i]
        #     agent = priorList.keys()[priorList.values().index(prior)]
        #     ambulanceStartConfig = agent.getStart()
        #     ambulanceGoalConfig = agent.getGoal()
        #     ambulance = agent.getRobot()
        #     multiPlan = multiPlanner.SIPP(ambulanceStartConfig, ambulanceGoalConfig, env, ambulance)
        #     multiPlan.TInterval = TI
        #     multiPlan.runSIPP()
        #     TI = multiPlanner.SIPP.TInterval
        removeAllRobots(env)
        ambulance = robotList[0]
        addIndividualRobot(env, ambulance, startConfigurations)
        ambulanceStartConfig = list(startConfigurations[ambulance])
        ambulanceGoalConfig = [2.8, 1.4, 0.05]
        multiPlan = multiPlanner.SIPP(ambulanceStartConfig, ambulanceGoalConfig, env, ambulance)
        multiPlan.runSIPP()
        for state in multiPlan.path:
            handler.append(env.plot3(np.array([state[0], state[1], 0.1]), 5.0,colors=np.array(((1, 1, 1)))))
        removeIndividualRobot(env, ambulance)

        ambulance = robotList[1]
        addIndividualRobot(env, ambulance, startConfigurations)
        ambulanceStartConfig = list(ambulance.GetActiveDOFValues())
        ambulanceGoalConfig = [0,-1.4,-pi/2]
        multiPlan1 = multiPlanner.SIPP(ambulanceStartConfig, ambulanceGoalConfig, env, ambulance)
        multiPlan1.TInterval = multiPlan.TInterval
        multiPlan1.runSIPP()
        removeIndividualRobot(env, ambulance)
        for state in multiPlan1.path:
            handler.append(env.plot3(np.array([state[0], state[1], 0.1]), 5.0, colors=np.array(((0, 0, 0)))))

        ambulance = robotList[2]
        addIndividualRobot(env, ambulance, startConfigurations)
        ambulanceStartConfig = list(ambulance.GetActiveDOFValues())
        ambulanceGoalConfig = [-2.8, 1.4, 0.05]
        multiPlan2 = multiPlanner.SIPP(ambulanceStartConfig, ambulanceGoalConfig, env, ambulance)
        multiPlan2.TInterval = multiPlan.TInterval
        multiPlan2.runSIPP()
        removeIndividualRobot(env, ambulance)
        addAllRobots(env, startConfigurations)
        for state in multiPlan2.path:
            handler.append(env.plot3(np.array([state[0], state[1], 0.1]), 5.0, colors=np.array(((0, 1, 0)))))
        path = multiPlan.path  # put your final path in this variable
        path1 = multiPlan1.path
        path2 = multiPlan2.path
        #### END OF YOUR CODE ###
        # Now that you have computed a path, convert it to an openrave trajectory
        # rob = robotList[0]
        # veh = robotList[1]
        # traj = ConvertPathToTrajectory(rob, path)
        # traj1 = ConvertPathToTrajectory(veh, path1)
        #
        # # Execute the trajectory on the robot.
        # if traj != None:
        #     rob.GetController().SetPath(traj)
        # if traj1 != None:
        #     veh.GetController().SetPath(traj1)

    waitrobot(robot)

    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")
    env.Destroy()
