#!/usr/bin/env python3


def VGV(group,base):

  import hebi
  import numpy as np
  from time import sleep
  import math


  # Populate variable 'current_position' with position feedback

  # Position, velocity, and acceleration waypoints.
  # Each column is a separate waypoint.
  # Each row is a different joint.
  num_joints=5
  pos = np.empty((num_joints, 15))
  vel = np.empty((num_joints, 15))
  acc = np.empty((num_joints, 15))

  # Set first and last waypoint values to 0.0
  vel[:,0] = acc[:,0] = 0.0
  vel[:,-1] = acc[:,-1] = 0.0
  # Set all other values to NaN
  vel[:,1:-1] = acc[:,1:-1] = np.nan

  # Set positions
  fbk = group_feedback
  group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)
  pos[:,0] = group_feedback.position
  #pos[:,1] = [base, -4.61, 2.06, 4.09, 0.0]
  pos[:,1] = [base, -3.32, 2.50, -2.38, 0.00]
  pos[:,2] = [base, -3.75, 2.10, -2.38, 0.00]
  pos[:,3] = [base, -4.26, 1.38, -2.57, 2.44]
  pos[:,4] = [base, -4.26, 1.38, -1.5, 3.99]
  pos[:,5] = [base, -4.26, 1.38, -0.28, 3.99]
  # Actuation
  pos[:,6] = [base, -4.26, 1.63, -0.28, 3.99]
  pos[:,7] = [base, -4.26, 1.63, -0.28, 3.5]
  pos[:,8] = [base, -4.26, 1.63, -0.28, 3.0]
  pos[:,9] = [base, -4.26, 1.47, -0.28, 2.44]
  pos[:,10] = [base, -4.26, 1.38, -0.28, 2.44]
  pos[:,11] = [base, -4.26, 1.38, -1.5, 2.44]
  pos[:,12] = [base, -4.26, 1.38, -2.57, 2.44]
  pos[:,13] = [base, -3.75, 2.10, -2.38, 0.00]
  pos[:,14] = [base, -3.32, 2.50, -2.38, 0.00]
  
  


  # The times to reach each waypoint (in seconds)
  time = np.linspace(0, 75, 15)

  # Define trajectory
  trajectory = hebi.trajectory.create_trajectory(time, pos, vel, acc)
  return trajectory
 
  
    


