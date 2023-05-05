def home(group,base):
  import hebi
  import numpy as np
  from time import sleep
  import math


  # Populate variable 'current_position' with position feedback

  # Position, velocity, and acceleration waypoints.
  # Each column is a separate waypoint.
  # Each row is a different joint.
  num_joints=5
  pos = np.empty((num_joints, 3))
  vel = np.empty((num_joints, 3))
  acc = np.empty((num_joints, 3))

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

  if (base==0.0):
    pos[:,0] = group_feedback.position
    pos[:,1] = [base, -4.7, 2.2, 4.15, 0.0] #Home3
    pos[:,2] = [base, -3.29, 2.83, 3.6, 0.0] #Home2
    
    home3 = [base, -4.7, 2.2, 4.15, 0.0]
  else:
    pos[:,0] = group_feedback.position
    pos[:,1] = [base, -4.7, 2.2, 4.15, 0.0] #Home3
    pos[:,2] = [0.0, -3.29, 2.83, 3.6, 0.0] #Home2

  # The times to reach each waypoint (in seconds)
  time = np.linspace(0, 15, 3)

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


