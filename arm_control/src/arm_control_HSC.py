#!/usr/bin/env python3

def HSC(group,base):
  # Endoscope read position
  
  import hebi
  import numpy as np
  from time import sleep
  import math

  lookup = hebi.Lookup()
    # Give the Lookup process 2 seconds to discover modules

  group = lookup.get_group_from_family('Arm')
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

  # Set feedback
  group_feedback = hebi.GroupFeedback(group.size)
  group.feedback_frequency = 200.0
  # Set positions
  fbk = group_feedback
  group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)
  
  pos[:,0] = group_feedback.position
  pos[:,1] = [base, -4.03, 2.23, 4.54, 0.0] #HSC-1
  pos[:,2] = [base, -4.03, 2.23, 4.54, 0.0] #HSC-1
  pos[:,3] = [base, -4.03, 2.23, 4.54, 0.0] #HSC-1
  pos[:,4] = [base, -4.06, 1.58, 4.86, 0.0] #HSC-2
  pos[:,5] = [base, -4.06, 1.58, 4.86, 0.0] #HSC-2
  pos[:,6] = [base, -4.06, 1.58, 4.86, 0.0] #HSC-2
  pos[:,7] = [base, -4.43, 1.25, 6.07, -0.9] #HSC-3
  pos[:,8] = [base, -4.43, 1.25, 6.07, -0.9] #HSC-4
  pos[:,9] = [base, -4.43, 1.25, 6.07, -0.9] #HSC-4
  pos[:,10] = [base, -4.43, 1.25, 6.07, -0.9] #HSC-4
  pos[:,11] = [base, -4.43, 1.25, 6.07, -0.9] #HSC-4


  # The times to reach each waypoint (in seconds)
  time = np.linspace(0, 40, 12)

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
    
def HSC_c2o(group,base):
  # Close to open from read position
  import hebi
  import numpy as np
  from time import sleep
  import math

  # Populate variable 'current_position' with position feedback

  lookup = hebi.Lookup()
    # Give the Lookup process 2 seconds to discover modules

  group = lookup.get_group_from_family('Arm')


  # Populate variable 'current_position' with position feedback

  # Position, velocity, and acceleration waypoints.
  # Each column is a separate waypoint.
  # Each row is a different joint.
  num_joints=5
  pos = np.empty((num_joints, 20))
  vel = np.empty((num_joints, 20))
  acc = np.empty((num_joints, 20))

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
  pos[:,1] = [base, -4.5, 2.1, 4.54, 0.0] #HSC-1
  pos[:,2] = [base, -4.18, 2.1, 4.54, 0.0] 
  pos[:,3] = [base, -4.08, 1.6, 5.0, 0.0]
  pos[:,4] = [base, -4.08, 1.37, 5.5, 0.0]
  pos[:,5] = [base, -4.08, 1.37, 6.2, 0.0]
  pos[:,6] = [base, -4.39, 1.15, 6.2, 0.0]
  pos[:,7] = [base+0.05, -4.39, 1.15, 6.2, 0.0]
  pos[:,8] = [base+0.05, -4.46, 1.15, 6.02, 0.0]
  pos[:,9] = [base+0.05, -4.56, 1.25, 6.02, 0.0]
  pos[:,10] = [base+0.05, -4.56, 1.25, 6.02, -1.13]
  #manipulation
  pos[:,11] = [base+0.05, -4.56, 1.25, 6.02, -2.13]
  pos[:,12] = [base+0.05, -4.56, 1.25, 6.02, -2.13]
  pos[:,13] = [base+0.05, -4.56, 1.35, 6.02, -3.13]
  pos[:,14] = [base+0.05, -3.91, 1.35, 6.02, -3.13]
  pos[:,15] = [base+0.05, -3.91, 1.25, 6.02, -3.13]
  pos[:,16] = [base+0.05, -3.91, 1.15, 6.02, -3.13]
  pos[:,17] = [base+0.05, -3.91, 1.8, 6.02, -3.13]
  pos[:,18] = [base, -3.29, 2.3, 3.6, 0.0]
  pos[:,19] = [0.0, -3.29, 2.83, 3.6, 0.0]





  # The times to reach each waypoint (in seconds)
  time = np.linspace(0, 50, 20)

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
    
def HSC_o2c(group,base):
  # Open to close from read position
  import hebi
  import numpy as np
  from time import sleep
  import math

  lookup = hebi.Lookup()
    # Give the Lookup process 2 seconds to discover modules

  group = lookup.get_group_from_family('Arm')


  # Populate variable 'current_position' with position feedback

  # Position, velocity, and acceleration waypoints.
  # Each column is a separate waypoint.
  # Each row is a different joint.
  num_joints=5
  pos = np.empty((num_joints, 19))
  vel = np.empty((num_joints, 19))
  acc = np.empty((num_joints, 19))

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

  # Set command timeout:
  group.command_lifetime = 200.00
  
  pos[:,0] = group_feedback.position
  pos[:,1] = [base, -4.5, 2.1, 4.54, 0.0] #HSC-1
  pos[:,2] = [base, -4.18, 2.1, 4.54, 0.0] 
  pos[:,3] = [base, -4.08, 1.6, 5.0, 0.0]
  pos[:,4] = [base, -4.08, 1.37, 5.5, 0.0]
  pos[:,5] = [base, -4.08, 1.37, 6.2, 0.0]
  pos[:,6] = [base, -4.39, 1.15, 6.2, 0.0]
  pos[:,7] = [base+0.05, -4.39, 1.15, 6.2, 0.0]
  pos[:,8] = [base+0.05, -4.46, 1.15, 6.02, 0.0]
  pos[:,9] = [base+0.05, -4.56, 1.25, 6.02, 1.13]
  pos[:,10] = [base+0.05, -4.56, 1.25, 6.02, 1.13]
  #manipulation
  pos[:,11] = [base+0.05, -4.56, 1.25, 6.02, 1.13]
  pos[:,12] = [base+0.05, -4.56, 1.25, 6.02, 1.13]
  pos[:,13] = [base+0.05, -4.56, 1.25, 6.02, 4.13]
  pos[:,14] = [base+0.05, -4.29, 1.25, 6.02, 4.4]
  pos[:,15] = [base+0.05, -4.29, 1.15, 6.02, 4.4]
  pos[:,16] = [base+0.05, -3.91, 1.6, 6.02, 1.13]
  pos[:,17] = [base, -3.59, 2.3, 3.6, 0.0]
  pos[:,18] = [0.0, -3.29, 2.83, 3.6, 0.0]


  # The times to reach each waypoint (in seconds)
  time = np.linspace(0, 60, 19)

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





