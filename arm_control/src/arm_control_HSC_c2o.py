#!/usr/bin/env python3

def HSC_c2o(group):

  import hebi
  import numpy as np
  from time import sleep
  import math


  # Populate variable 'current_position' with position feedback

  # Position, velocity, and acceleration waypoints.
  # Each column is a separate waypoint.
  # Each row is a different joint.
  num_joints=5
  pos = np.empty((num_joints, 9))
  vel = np.empty((num_joints, 9))
  acc = np.empty((num_joints, 9))

  # Set first and last waypoint values to 0.0
  vel[:,0] = acc[:,0] = 0.0
  vel[:,-1] = acc[:,-1] = 0.0
  # Set all other values to NaN
  vel[:,1:-1] = acc[:,1:-1] = np.nan

  # Set positions
  fbk = group_feedback
  group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)
  pos[:,0] = group_feedback.position
  #pos[:,1] = [0.0, -3.29, 2.83, 4.09, 0.0] #Home2
  #pos[:,2] = [0.0, -4.61, 2.06, 4.09, 0.0] #Home3
  pos[:,1] = [0.0, -4.03, 2.23, 4.54, 0.0] #HSC-1
  pos[:,2] = [0.0, -4.06, 1.52, 4.86, 3.14] #HSC-2
  pos[:,3] = [0.0, -4.33, 1.53, 6.07, 3.14] #HSC-3
  pos[:,4] = [0.0, -4.35, 1.58, 6.07, -0.8] #HSC-4
  pos[:,5] = [0.0, -4.25, 1.56, 6.07, -0.8] #HSC-5 -4.35
  pos[:,6] = [0.0, -3.93, 1.48, 6.07, -0.] #HSC-4
  #pos[:,7] = [0.0, -4.43, 1.35, 6.07, -0.4] #HSC-3
  #pos[:,8] = [0.0, -3.98, 1.35, 4.86, -0.4] #HSC-2
  pos[:,7] = [0.0, -3.5, 2.23, 4.54, -0.6] #HSC-1
  pos[:,8] = [0.0, -3.29, 2.83, 4.09, 0.0] #Home2


  # The times to reach each waypoint (in seconds)
  time = np.linspace(0, 60, 9)

  # Define trajectory
  trajectory = hebi.trajectory.create_trajectory(time, pos, vel, acc)

  cmd = hebi.GroupCommand(num_joints)
  period = 0.01
  duration = trajectory.duration

  pos_cmd = np.array(num_joints, dtype=np.float64)
  vel_cmd = np.array(num_joints, dtype=np.float64)

  t = 0.0

  while (t < duration):
    pos_cmd, vel_cmd, acc_cmd = trajectory.get_state(t)
    cmd.position = pos_cmd
    cmd.velocity = vel_cmd
    group.send_command(cmd)

    t = t + period
    sleep(period)
    


