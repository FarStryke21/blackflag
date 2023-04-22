#!/usr/bin/env python3

import arm_control_HSC_c2o
import arm_control_HSC_c2o_v2
import arm_control_HSC_o2c
import arm_control_VSC_c2o
import arm_control_VSC_o2c
import arm_control_vision
import arm_control_Breaker
import arm_control_LV
import arm_control_VGV
import arm_control_HGV
import math
import hebi
import numpy as np
from time import sleep

lookup = hebi.Lookup()
# Give the Lookup process 2 seconds to discover modules
sleep(2)
print('Modules found on network:')
for entry in lookup.entrylist:
    print(f'{entry.family} | {entry.name}')

group = lookup.get_group_from_family('Arm')


# Send gains:
group_cmd = hebi.GroupCommand(group.size)
group_cmd.read_gains("correct_gains.xml")
group.send_command_with_acknowledgement(group_cmd)

valve='hsc_o2c'

base = 0.0
arm_control_vision.vision(group, base)
# for station 6 base should be -1.27

num_joints = 5

if(valve=='lv'):
    trajectory=arm_control_LV.LV(group,base)
elif(valve=='hsc_o2c'):
    trajectory=arm_control_HSC_o2c.HSC_o2c(group,base)
elif(valve=='hsc_c2o'):
    trajectory=arm_control_HSC_c2o_v2.HSC_c2o_v2(group, base)
elif(valve=='vsc_o2c'):
    trajectory=arm_control_VSC_o2c.VSC_o2c(group,base)
elif(valve=='vsc_c2o'):
    trajectory=arm_control_VSC_c2o.VSC_c2o(group,base)
elif(valve=='vgv'):
    trajectory=arm_control_VGV.VGV(group,base)
elif(valve=='hgv'):
    trajectory=arm_control_HGV.HGV(group,base)
    
    



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
