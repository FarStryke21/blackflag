#!/usr/bin/env python3

def breaker(group, mission_info, base=0.0):
  import hebi
  import numpy as np
  from time import sleep
  import math

  num_joints=5
  pos = np.empty((num_joints, 11))
  vel = np.empty((num_joints, 11))
  acc = np.empty((num_joints, 11))

  # Set first and last waypoint values to 0.0
  vel[:,0] = acc[:,0] = 0.0
  vel[:,-1] = acc[:,-1] = 0.0
  # Set all other values to NaN
  vel[:,1:-1] = acc[:,1:-1] = np.nan

  lookup = hebi.Lookup()
    # Give the Lookup process 2 seconds to discover modules

  group = lookup.get_group_from_family('Arm')

  group_feedback = hebi.GroupFeedback(group.size)
  group.feedback_frequency = 200.0
  # Set positions
  fbk = group_feedback
  group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)

  # Set command timeout:
  group.command_lifetime = 200.00

  if (base==0.0):
    pos[:,0] = group_feedback.position
    pos[:,1] = [base, -3.96, 2.19, 4.87, 0.0]
    pos[:,2] = [base, -3.96, 2.19, 1.46, 0.0]
    pos[:,3] = [base, -4.07, 2.09, 1.46, 0.0]
    pos[:,4] = [base, -4.07, 2.09, 1.46, -2.29]
    pos[:,5] = [base+0.1, -4.07, 2.09, 1.46, -2.29]
    pos[:,6] = [base+0.1, -3.93, 2.08, 1.46, -1.69]
    pos[:,7] = [base+0.1, -4.10, 1.98, 1.46, -1.69]
    pos[:,8] = [base+0.1, -4.10, 1.98, 1.22, -1.69]
    pos[:,9] = [base+0.1, -4.10, 1.98, 1.22, -1.69]
    pos[:,10] = [base+0.1, -4.10, 1.98, 1.22, -1.69]


  # The times to reach each waypoint (in seconds)
  time = np.linspace(0, 40, 11)

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


