#!/usr/bin/env python3

def VSC_c2o(group,base=0.0):
  import hebi
  import numpy as np
  from time import sleep
  import math

 
  
  # Populate variable 'current_position' with position feedback

  # Position, velocity, and acceleration waypoints.
  # Each column is a separate waypoint.
  # Each row is a different joint.
  num_joints=5
  pos = np.empty((num_joints, 17))
  vel = np.empty((num_joints, 17))
  acc = np.empty((num_joints, 17))

  # Set first and last waypoint values to 0.0
  vel[:,0] = acc[:,0] = 0.0
  vel[:,-1] = acc[:,-1] = 0.0
  # Set all other values to NaN
  vel[:,1:-1] = acc[:,1:-1] = np.nan

  # Set positions
  fbk = group_feedback
  group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)
  pos[:,0] = group_feedback.position
  pos[:,1] = [base, -3.29, 2.83, 4.09, 0.0] #Home2
  pos[:,2] = [base, -4.51, 2.2, 4.87, 0.0] #Home3
  pos[:,3] = [base, -3.93, 1.98, 4.87, 0.0] #VSC-1
  pos[:,4] = [base, -3.93, 1.98, 4.87, 0.0] #VSC-2
  pos[:,5] = [base, -3.93, 1.98, 0.94, 0.0] #VSC-3
  pos[:,6] = [base, -4.04, 2.06, 1.71, 0.0] #VSC-4
  pos[:,7] = [base, -4.04, 2.06, 1.74, 0.0] #VSC-5
  pos[:,8] = [base, -4.04, 1.89, 1.74, -2.75] #VSC-6
  pos[:,9] = [base, -4.04, 1.89, 1.74, -2.9] #VSC-7
  pos[:,10] = [base, -4.04, 1.89, 1.74, -2.75] #VSC-6
  pos[:,11] = [base, -4.04, 2.06, 1.74, 0.0] #VSC-5
  pos[:,12] = [base, -4.09, 2.06, 1.71, 0] #VSC-4
  pos[:,13] = [base, -3.93, 1.98, 0.94, 0] #VSC-3
  pos[:,14] = [base, -3.93, 1.98, 4.87, 0] #VSC-2
  pos[:,15] = [base, -3.93, 1.98, 4.87, 0] #VSC-1
  pos[:,16] = [base, -3.29, 2.83, 4.09, 0.0] #Home2   

  # The times to reach each waypoint (in seconds)
  time = np.linspace(0, 80, 17)

  # Define trajectory
  trajectory = hebi.trajectory.create_trajectory(time, pos, vel, acc)

  return trajectory

