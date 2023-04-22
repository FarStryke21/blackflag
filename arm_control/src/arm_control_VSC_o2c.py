#!/usr/bin/env python3

def VSC_o2c(group,base=0.0):
  import hebi
  import numpy as np
  from time import sleep
  import math

  # Populate variable 'current_position' with position feedback

  # Position, velocity, and acceleration waypoints.
  # Each column is a separate waypoint.
  # Each row is a different joint.
  num_joints=5
  pos = np.empty((num_joints, 12))
  vel = np.empty((num_joints, 12))
  acc = np.empty((num_joints, 12))

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
  pos[:,3] = [base, -3.93, 1.98, 4.87, -3.6] #VSC-1
  pos[:,4] = [base, -3.93, 1.98, 0.94, -3.6] #VSC-2
  pos[:,5] = [base+0.1, -4.05, 2, 1.6, -1.5] #VSC-3
  # pos[:,6] = [base+0.1, -4.05, 2, 1.8, 0]
  # pos[:,7] = [base+0.1, -4.05, 2, 1.8, 0]
  # pos[:,8] = [base, -3.93, 1.98, 0.94, -3.6] #VSC-3
  # pos[:,9] = [base, -3.93, 1.98, 4.87, -3.6] #VSC-1
  # pos[:,10] = [base, -4.51, 2.2, 4.87, 0.0] #Home3
  # pos[:,11] = [base, -3.29, 2.83, 4.09, 0.0] #Home2

  # The times to reach each waypoint (in seconds)
  time = np.linspace(0, 86, 12)

  # Define trajectory
  trajectory = hebi.trajectory.create_trajectory(time, pos, vel, acc)

  return trajectory

