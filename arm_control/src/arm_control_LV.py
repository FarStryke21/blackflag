#!/usr/bin/env python3
import arm_control_home

def LV(group,base,mission_info):

  import hebi
  import numpy as np
  from time import sleep
  import math


  # Populate variable 'current_position' with position feedback

  lookup = hebi.Lookup()
  # Give the Lookup process 2 seconds to discover modules

  group = lookup.get_group_from_family('Arm')

  # Position, velocity, and acceleration waypoints.
  # Each column is a separate waypoint.
  # Each row is a different joint.
  num_joints=5
  pos = np.empty((num_joints, 16))
  vel = np.empty((num_joints, 16))
  acc = np.empty((num_joints, 16))

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
  group.command_lifetime = 200.00
  #-0.15 if at 4
  pos[:,0] = group_feedback.position
  pos[:,1] = [base, -4.61, 2.06, 4.09, 0.0] 
  pos[:,2] = [base, -4.03, 2.2, 4.54, 0.0] 
  pos[:,3] = [base, -3.8, 1.7, 4.96, 0.0]
  pos[:,4] = [base+0.1, -3.8, 1.7, 4.96, 0.0]
  pos[:,5] = [base+0.1, -3.95, 1.92, 1.17, 0.0]
  pos[:,6] = [base+0.1, -3.95, 2.01, 1.4, 0.0]
  pos[:,7] = [base+0.1, -3.95, 2.01, 1.4, 0.0]
  pos[:,8] = [base+0.1, -3.95, 2.01, 1.44, 0.0]
  pos[:,9] = [base+0.1, -4.4, 2.01, 1.44, 0.0]
  pos[:,10] = [base+0.1, -4.4, 2.01, 1.44, 0.0]
  pos[:,11] = [base+0.1, -4.3, 2.01, 1.44, 2.25]
  pos[:,12] = [base+0.1, -4.3, 2.01, 1.44, 2.25]
  pos[:,13] = [base+0.1, -3.95, 2.01, 1.44, 2.25]
  pos[:,14] = [base+0.1, -3.8, 2.01, 1.44, 2.25]
  pos[:,15] = [base+0.1, -3.8, 2.3, 1.44, 2.25]



  # The times to reach each waypoint (in seconds)
  time = np.linspace(0, 60, 16)

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

