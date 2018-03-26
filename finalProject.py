#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
from planner import priorityPlanner as prp

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

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
    env.Load('./data/4PR2_door.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    for i in range(len(env.GetRobots())):
         robot = env.GetRobots()[i]
         tuckarms(env, robot)
         robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis, [0, 0, 1])
         # tuck in the PR2's arms for driving

    # Robot[0] is the ambulance and all others are vehicles
    ambulance = env.GetRobots()[0]

    ambulanceStartConfig = list(ambulance.GetActiveDOFValues())
    ambulanceGoalConfig = [4.60594,0.28479,0]

    #### YOUR CODE HERE ####
    prp.priorityPlanner(env, ambulance, ambulanceStartConfig, ambulanceGoalConfig)






    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")
    env.Destroy()
