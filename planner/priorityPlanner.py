import numpy
import planner as backward
import SIPP

def priorityPlanner(env, robot, vehicles, start, goal):
    backwardPlanner = backward.BiRRTConnect(start, goal, maxIter, step, env, robot, smooth=True)
    canPlan = False             #Currently we can not plan
    while canPlan == False:     #While we do not have any plan for the backward route for the ambulance. We try to find one
        status = backwardPlanner.buildBiRRTConnect()
        if status == "success":
            canPlan = True
    obstacles = []
    path = backwardPlanner.getPath()
    for vehicle in vehicles:
        vehicleConfig = vehicle.GetActiveDOFValues()
        for index in range(path):
            if SIPP.euclideanMetric(vehicleConfig, path[index]) < 0.5:
                obstacles.append(vehicle)
                break
