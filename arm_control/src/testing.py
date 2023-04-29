#!/usr/bin/env python

import hebi
import numpy as np
from time import sleep
import math

lookup = hebi.Lookup()
# Give the Lookup process 2 seconds to discover modules
sleep(2)
print('Modules found on network:')
for entry in lookup.entrylist:
    print(f'{entry.family} | {entry.name}')

group = lookup.get_group_from_family('Arm')

# Send gains:
group_cmd = hebi.GroupCommand(group.size)
group_cmd.read_gains("Arm control/correct_gains.xml")
group.send_command_with_acknowledgement(group_cmd)

# Get Feedback from the modules:
# Best practice is to allocate this once, not every loop iteration
group_feedback = hebi.GroupFeedback(group.size)

# This effectively limits the loop below to 200Hz
group.feedback_frequency = 200.0
# Populate variable 'current_position' with position feedback

# Position, velocity, and acceleration waypoints.
# Each column is a separate waypoint.
# Each row is a different joint.
num_joints=5
pos = np.empty((num_joints, 6))
vel = np.empty((num_joints, 6))
acc = np.empty((num_joints, 6))

# Set first and last waypoint values to 0.0
vel[:,0] = acc[:,0] = 0.0
vel[:,-1] = acc[:,-1] = 0.0
# Set all other values to NaN
vel[:,1:-1] = acc[:,1:-1] = np.nan

# Set positions
fbk = group_feedback
group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)
pos[:,0] = group_feedback.position
pos[:,1] = [0.0, -3.29, 2.83, 4.09, 0.0] #Home2
pos[:,2] = [0.0, -4.7, 2.2, 4.87, 0.0]
pos[:,3] = [0.0, -4.7, 2.2, 4.87, 0.0]
pos[:,4] = [0.0, -4.7, 2.2, 4.87, 0.0]
pos[:,5] = [0.0, -4.03, 2.23, 4.54, 0.0] #HSC-1



# The times to reach each waypoint (in seconds)
time = np.linspace(0, 60, 6)

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
