#!/usr/bin/env python

import arm_control_HSC
import arm_control_HGV
import arm_control_Breaker
import arm_control_LV
import arm_control_VGV
import arm_control_VSC
import arm_control_home
import panel_classifier

import arm_control_vision

import hebi
import numpy as np
from time import sleep
import rospy
from std_msgs.msg import String, Bool, Int32
import os


def current_panel(msg):
    global current_valve
    if not msg.data == '':
        current_valve = msg.data

def panel_info_callback(msg):
    global mission_panel
    global mission_info

    data = msg.data
    data = data.split(':')
    mission_panel = data[0].lower()
    mission_info = str(data[1])

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
    reached = 1 # Intially at correct position
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
    # for entry in lookup.entrylist:
    #     print(f'{entry.family} | {entry.name}')

    group = lookup.get_group_from_family('Arm')


    # Send gains:
    config_dir = '/home/mechatronics/catkin_ws/src/blackflag/arm_control/config'
    gain_file = os.path.join(config_dir, "correct_gains.xml")
    group_cmd = hebi.GroupCommand(group.size)
    group_cmd.read_gains(gain_file)
    group.send_command_with_acknowledgement(group_cmd)
    # valid_results = ['horizontal stopcock:open',
    #                  'horizontal stopcock:close',
    #                  'vertical stopcock:open',
    #                  'vertical stopcock:close',
    #                  'breaker',
    #                  'vertical gate valve',
    #                  'horizontal gate valve',
    #                  'large valve']
    valid_results = ['stopcock', 'gate valve', 'breaker', 'large valve']
    ##-------------------------------------------------
    for ch in mission_code:
        if ch == '1':
            while True:
                if (reached == 1):
                    reached = -1
                    break
            # rospy.loginfo(f'Starting loop {i+1}')

            if i < 5:
                base = 0.0
            else:
                base = -1.57

            arm_control_vision.vision(group, base)

            current_valve = ''

            num_joints = 5
            trajectory = 0

            classification = ''
            task = ''
            looking = ''

            while True:
                if (mission_panel in current_valve and mission_panel in valid_results):
                    classification = mission_panel
                    task = mission_info
                    looking = current_valve
                # if current_valve in valid_results:
                #     classification = current_valve
                    break
                
            looking = looking.split('_')

            if((classification == 'large valve') and ('large valve' in looking)):
                arm_control_LV.LV(group, base, int(task))
    
            elif(('horizontal stopcock:close' in looking) and ('stopcock' == classification) and (task == '0')):
            # elif(classification =='horizontal stopcock:close'):
                arm_control_HSC.HSC_c2o(group, base)
            
            elif(('horizontal stopcock:open' in looking) and ('stopcock' == classification) and (task == '1')):
            # elif(classification=='horizontal stopcock:open'):
                arm_control_HSC.HSC_o2c(group, base)
            
            elif(('vertical stopcock:close' in looking) and ('stopcock' == classification) and (task == '0')):
            # elif(classification=='vertical stopcock:close'):
                arm_control_VSC.VSC_c2o(group, base)
            
            elif(('vertical stopcock:open' in looking) and ('stopcock' == classification) and (task == '1')):
            # elif(classification=='vertical stopcock:open'):
                arm_control_VSC.VSC_o2c(group, base)
                
            elif(('vertical gate valve' in looking) and (classification=='gate valve')):
            # elif(classification=='vertical gate valve'):
                arm_control_VGV.VGV(group, base, int(task))
                
            elif(('horizontal gate valve' in looking) and (classification=='gate valve')):
            # elif(classification=='horizontal gate valve'):
                arm_control_HGV.HGV(group, base, int(task))

            elif((classification=='breaker') and ('breaker' in looking)):
                # arm_control_Breaker.breaker(group, base, mission_info)
                arm_control_home.home(group, base)

            else:
                arm_control_home.home(group, base)


            try:
                msg = Bool()
                msg.data = True
                completition_status_publisher.publish(msg)
            except rospy.ROSInterruptException:
                pass
                

        # rospy.spin()
    ##-------------------------------------------------