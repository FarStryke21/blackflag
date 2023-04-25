#!/usr/bin/env python3
import arm_control_HSC
import arm_control_HGV
import arm_control_Breaker
import arm_control_LV
import arm_control_VGV
import arm_control_VSC
import arm_control_home
import panel_classifier

import arm_control_vision

import math
import hebi
import numpy as np
from time import sleep
import rospy
from std_msgs.msg import String, Bool, Int32
from sensor_msgs.msg import Image
import os
from cv_bridge import CvBridge
import cv2

def current_panel(msg):
    global current_valve
    current_valve = msg.data

def panel_info_callback(msg):
    global mission_panel
    global mission_info

    data = msg.data
    data = data.split(':')
    mission_panel = data[0].lower()
    mission_info = data[1]

def mission_code_callback(msg):
    global mission_code
    global flag
    if flag and msg.data != '':
        mission_code = msg.data
        flag = False

def counter_callback(msg):
    global i
    i = int(msg.data)

def reached_callback(msg):
    global reached
    reached = 1

if __name__ == '__main__':
    
    current_valve = ''
    mission_panel = ''
    mission_info = ''
    img = 0
    cropped_img = 0
    reached = 0 # Intially at correct position
    flag = True
    i = -1
    rospy.init_node('arm_node', anonymous=True)
    rate = rospy.Rate(30)
    rospy.Subscriber('/realsense_predict', String, current_panel)
    rospy.Subscriber('/panel_info', String, panel_info_callback)
    rospy.Subscriber('/mission_code', String, mission_code_callback)
    rospy.Subscriber('/reached_status', Int32, reached_callback)
    rospy.Subscriber('/counter_status', Int32, counter_callback)
    completition_status_publisher = rospy.Publisher('/completion_status', Bool, queue_size = 10)
    current_valve_publisher = rospy.Publisher('/current_valve', String, queue_size = 10)

    lookup = hebi.Lookup()
    # Give the Lookup process 2 seconds to discover modules
    sleep(2)
    print('Modules found on network:')
    for entry in lookup.entrylist:
        print(f'{entry.family} | {entry.name}')

    group = lookup.get_group_from_family('Arm')


    # Send gains:
    config_dir = '/home/aman/catkin_ws/src/blackflag/arm_control/config'
    gain_file = os.path.join(config_dir, "correct_gains.xml")
    group_cmd = hebi.GroupCommand(group.size)
    group_cmd.read_gains(gain_file)
    group.send_command_with_acknowledgement(group_cmd)
    valid_results = ['horizontal stopcock:open',
                     'horizontal stopcock:close',
                     'vertical stopcock:open',
                     'vertical stopcock:close',
                     'breaker',
                     'vertical gate valve',
                     'horizontal gate valve',
                     'large valve']
    ##-------------------------------------------------
    for ch in mission_code:
        if ch == '1':
            rospy.loginfo(f'Starting loop {i+1}')

            if i < 5:
                base = 0.0
            else:
                base = -1.57

            arm_control_vision.vision(group, base)

            num_joints = 5
            trajectory = 0
            classification = ''

            while True:
                if (current_valve in valid_results):
                    classification = current_valve
                    break

            if(classification == 'large valve'):
                arm_control_LV.LV(group, base, mission_info)
    
            elif('horizontal stopcock:close' == classification):
                arm_control_HSC.HSC_c2o(group, base)
            
            elif('horizontal stopcock:open' == classification):
                arm_control_HSC.HSC_o2c(group, base)
            
            elif('vertical stopcock:close' == classification):
                arm_control_VSC.VSC_c2o(group, base)
            
            elif('vertical stopcock:open' == classification):
                arm_control_VSC.VSC_o2c(group, base)
                
            elif(classification=='vertical gate valve'):
                arm_control_VGV.VGV(group, base, mission_info)
                
            elif(classification=='horizontal gate valve'):
                arm_control_HGV.HGV(group, base, mission_info)

            elif(classification=='breaker'):
                arm_control_Breaker.breaker(group, base, mission_info)

            # cmd = hebi.GroupCommand(num_joints)
            # period = 0.01
            # duration = trajectory.duration

            # pos_cmd = np.array(num_joints, dtype=np.float64)
            # vel_cmd = np.array(num_joints, dtype=np.float64)

            # t = 0.0

            # while (t < duration):
            #     pos_cmd, vel_cmd, acc_cmd = trajectory.get_state(t)
            #     cmd.position = pos_cmd
            #     cmd.velocity = vel_cmd
            #     group.send_command(cmd)

            #     t = t + period
            #     sleep(period)

            try:
                msg = Bool()
                msg.data = True
                completition_status_publisher.publish(msg)
            except rospy.ROSInterruptException:
                pass
                
            while True:
                if (reached == 1):
                    reached = -1
                    break

        # rospy.spin()
    ##-------------------------------------------------