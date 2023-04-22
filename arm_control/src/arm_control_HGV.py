#!/usr/bin/env python3


def HGV(group,base):

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
  pos[:,1] = [base, -4.61, 2.06, 4.09, 0.0]
  pos[:,2] = [base, -4.34, 2.35, 4.09, 0.0]
  pos[:,3] = [base+0.1, -3.82, 2.35, 4.09, 0.0]
  pos[:,4] = [base+0.15, -3.82, 2.07, 1.74, 0.0]
  pos[:,5] = [base+0.15, -3.75, 2.07, 1.74, 0.0]
  pos[:,6] = [base+0.15, -3.75, 2.07, 1.74, 0.0]
  pos[:,7] = [base+0.15, -3.94, 2.07, 1.36, 0.0]
  pos[:,8] = [base+0.15, -4, 2.07, 1.4, 0.0]
  # Actuation
  pos[:,9] = [base+0.15, -4, 2.07, 1.4, -2.9]
  pos[:,10] = [base+0.15, -3.75, 2.07, 1.4, -2.9]
  pos[:,11] = [-0.03, -3.29, 2.83, 4.09, 0.0] #Home2

  #pos[:,1] = [base, -4.61, 2.06, 4.09, 0.0]



  # The times to reach each waypoint (in seconds)
  time = np.linspace(0, 60, 12)

  # Define trajectory
  trajectory = hebi.trajectory.create_trajectory(time, pos, vel, acc)

  return trajectory


    


