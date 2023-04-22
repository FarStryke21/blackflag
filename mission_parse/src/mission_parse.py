#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
import os


def code_publish(plan):
    rospy.loginfo('Encoded Code Publishing started!')
    while not rospy.is_shutdown():
        code_pub.publish(plan)
        rate.sleep()

def parse_mission(filename):
    # Store mission files in the directory below
    filedirectory = '/home/aman/catkin_ws/src/blackflag/mission_parse/samples/mission_files'
    
    filepath = os.path.join(filedirectory,filename)
    mission = open(filepath, 'r+')
    mission_data = mission.readline()

    segments = mission_data.split(",")
    segments = [segment.strip() for segment in segments]
    time = int(segments[-1])
    segments = segments[0:-1]

    feature_map = {'V1':'Gate Valve', 'V2':'Large Valve', 'V3':'Stopcock Valve', 'A':'Breaker Box', 'B':'Breaker Box'}

    parsed_command = []

    for segment in segments:
        command = []
        command.append(segment[0])
        feature = segment[1] if segment[1]=='A' or segment[1] == 'B' else segment[1:3]
        command.append(feature_map[feature])
        if feature=='A' or feature=='B':
            info = segment.split(" ")
            button = [0,0,0]
            info_button = 1 if info[2] == 'U' else -1
            button[int(info[1][1])-1] = info_button
            command.append(button)
        else:
            info = segment.split(" ")
            command.append(int(info[1]))
        parsed_command.append(command)
        
    parsed_command = sorted(parsed_command, key = lambda x: x[0])
    return parsed_command, time

def encode(plan):
    code = [task[0] for task in plan]
    encoded_plan = '00000000'
    ch = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H']
    for i in range(0,8):
        if ch[i] in code:
            encoded_plan = encoded_plan[:i]+'1'+encoded_plan[i+1:]
                
    return encoded_plan

def callback(data):
    global loc
    task = parsed_command[loc]
    message = str(task[1])+":"+str(task[2])
    if loc < len(parsed_command)-1:
        loc = loc+1
        rospy.loginfo(f'Publishing data for {loc}th element in mission file')
        data_pub.publish(message)
    else:
        rospy.loginfo('End of File reached!')

if __name__ == '__main__':
    # filename = input('Enter File name : ')
    filename = 'hard.txt'
    parsed_command, time = parse_mission(filename)
    encoded_plan = encode(parsed_command)
    loc = 0
    try:
        code_pub = rospy.Publisher('mission_code', String, queue_size=10)
        data_pub = rospy.Publisher('panel_info', String, queue_size=10)
        rospy.Subscriber("completion_status", Bool, callback)
        rospy.init_node('mission_parse', anonymous=True)

        callback(True)
        rate = rospy.Rate(10)
        code_publish(encoded_plan)

    except rospy.ROSInterruptException:
        pass

