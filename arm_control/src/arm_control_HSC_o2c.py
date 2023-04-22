#!/usr/bin/env python3

def HSC_o2c(group,base):

  import hebi
  import numpy as np
  from time import sleep
  import math


  # Populate variable 'current_position' with position feedback

  # Position, velocity, and acceleration waypoints.
  # Each column is a separate waypoint.
  # Each row is a different joint.
  num_joints=5
  pos = np.empty((num_joints, 11))
  vel = np.empty((num_joints, 11))
  acc = np.empty((num_joints, 11))

  # Set first and last waypoint values to 0.0
  vel[:,0] = acc[:,0] = 0.0
  vel[:,-1] = acc[:,-1] = 0.0
  # Set all other values to NaN
  vel[:,1:-1] = acc[:,1:-1] = np.nan

   # Set feedback
  group_feedback = hebi.GroupFeedback(group.size)
  group.feedback_frequency = 200.0
  # Set positions
  fbk = group_feedback
  group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)
  pos[:,0] = group_feedback.position
  #pos[:,1] = [0.0, -3.29, 2.83, 4.09, 0.0] #Home2
  #pos[:,2] = [0.0, -4.61, 2.06, 4.09, 0.0] #Home3
  pos[:,1] = [base, -4.03, 2.23, 4.54, 0.0] #HSC-1
  pos[:,2] = [base, -4.06, 1.58, 4.86, 0.0] #HSC-2
  pos[:,3] = [base, -4.43, 1.25, 6.07, -0.9] #HSC-3
  pos[:,4] = [base, -4.43, 1.25, 6.07, -0.9] #HSC-4
  pos[:,5] = [base, -4.47, 1.42, 6.07, -0.9] #HSC-5
  pos[:,6] = [base, -4.47, 1.42, 6.07, 1.24] #HSC-4
  pos[:,7] = [base, -4.47, 1.25, 6.07, 1.24] #HSC-4
  pos[:,8] = [base, -4.23, 1.25, 4.86, 1.24] #HSC-2
  pos[:,9] = [base, -3.8, 2.23, 4.54, 0.0] #HSC-1
  pos[:,10] = [base, -3.29, 2.83, 4.09, 0.0] #Home2


  # The times to reach each waypoint (in seconds)
  time = np.linspace(0, 30, 11)

  # Define trajectory
  trajectory = hebi.trajectory.create_trajectory(time, pos, vel, acc)

  return trajectory


