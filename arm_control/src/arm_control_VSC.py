#!/usr/bin/env python3

import arm_control_home

def VSC_c2o(group,base=0.0):
  import hebi
  import numpy as np
  from time import sleep
  import math

 
  
  # Populate variable 'current_position' with position feedback
  # Populate variable 'current_position' with position feedback
  lookup = hebi.Lookup()
    # Give the Lookup process 2 seconds to discover modules

  group = lookup.get_group_from_family('Arm')


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

  group_feedback = hebi.GroupFeedback(group.size)
  group.feedback_frequency = 200.0
  # Set positions
  fbk = group_feedback
  group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)

  # Set command timeout:
  group.command_lifetime = 0.00

  pos[:,0] = group_feedback.position
  pos[:,1] = [base, -4.7, 2.2, 4.15, 0.0] #VSC-1
  pos[:,2] = [base, -4.09, 2.2, 4.14, 0.0]
  pos[:,3] = [base, -4.09, 2.2, 4.14, 0.0]
  pos[:,4] = [base, -4.09, 2.2, 3.60, 0.0]
  pos[:,5] = [base, -4.09, 2.2, 3.20, 0.0]
  pos[:,6] = [base, -4.09, 2.2, 2.8, 0.0]
  pos[:,7] = [base, -4.09, 2.2, 2.3, 0.0]
  pos[:,8] = [base, -4.09, 2.2, 2.0, 0.0]
  pos[:,9] = [base, -4.09, 2.2, 1.8, -0.88]
  pos[:,10] = [base, -4.09, 2.2, 1.53, -0.88]
  pos[:,11] = [base, -4.27, 2.2, 1.53, -0.88]
  pos[:,12] = [base, -4.27, 1.94, 1.53, -1.58]
  pos[:,13] = [base, -4.27, 1.94, 1.53, -1.58]
  pos[:,14] = [base, -4.27, 1.73, 1.53, -1.58]
  pos[:,15] = [base+0.1, -4.27, 1.94, 1.53, -1.58]
  pos[:,16] = [0.0, -4.27, 1.94, 1.53, -1.58]

  # The times to reach each waypoint (in seconds)
  time = np.linspace(0, 50, 17)

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

  arm_control_home.home(group, base)

def VSC_o2c(group,base=0.0):
  import hebi
  import numpy as np
  from time import sleep
  import math

  # Populate variable 'current_position' with position feedback
  # Populate variable 'current_position' with position feedback
  lookup = hebi.Lookup()
    # Give the Lookup process 2 seconds to discover modules

  group = lookup.get_group_from_family('Arm')

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

  group_feedback = hebi.GroupFeedback(group.size)
  group.feedback_frequency = 200.0
  # Set positions
  fbk = group_feedback
  group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)

  # Set command timeout:
  group.command_lifetime = 0.00
  
  pos[:,0] = group_feedback.position
  pos[:,1] = [base, -4.7, 2.2, 4.15, 0.0] #VSC-1
  pos[:,2] = [base, -4.09, 2.2, 4.14, 0.0]
  pos[:,3] = [base, -4.09, 2.2, 4.14, 0.0]
  pos[:,4] = [base, -4.0, 2.2, 3.60, 0.0]
  pos[:,5] = [base, -4.0, 2.2, 3.20, 0.0]
  pos[:,6] = [base+0.22, -4.0, 2.2, 2.8, 0.0]
  pos[:,7] = [base+0.22, -4.0, 2, 2.3, -0.8]
  pos[:,8] = [base+0.22, -4.09, 1.9, 2, -0.8]
  pos[:,9] = [base+0.22, -4.09, 1.85, 1.45, -0.8]
  pos[:,10] = [base, -4.09, 1.85, 1.45, -0.8]
  pos[:,11] = [base, -4.09, 1.85, 1.45, -0.8]
  pos[:,12] = [base+0.22, -4.2, 1.8, 1.45, -0.8]
  pos[:,13] = [base+0.22, -4.2, 1.8, 1.45, -0.8]
  pos[:,14] = [base+0.28, -4.2, 1.8, 1.45, -0.8]
  pos[:,15] = [base+0.28, -4.2, 1.8, 1.45, -0.8]
  pos[:,16] = [0.0, -3.29, 2.83, 3.6, 0.0]

  
  # The times to reach each waypoint (in seconds)
  time = np.linspace(0, 45, 17)

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

