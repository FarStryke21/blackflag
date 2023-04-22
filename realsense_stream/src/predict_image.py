#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from cv_bridge import CvBridge
import os
import torch
import numpy as np
from roboflow import Roboflow
import roboflow

def callback(image_msg):
    # Convert ROS image
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_msg, desired_encoding = 'passthrough')
    cv_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    prediction(cv_image)
    # cv2.imshow('Image', cv_image)
    # cv2.waitKey(1)

def processing(frame, outputs):
    return_frame = frame
    labels = []
    xs = []
    try :
        for output in outputs:
            x = output['x']
            y = output['y']
            xs.append(x)
            w = output['width']
            h = output['height']
            return_frame = cv2.rectangle(return_frame, (int(x - w/2), int(y - h/2)), (int(x + w/2), int(y + h/2)), (0,0,255), 2)
            label = output['class']
            labels.append(label)
            return_frame = cv2.putText(return_frame, label, (int(x - w/2), int(y - h/2) - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
    except Exception:
        pass

    return return_frame, labels, xs

def prediction(image):
    # frame = np.asanyarray(image)
    outputs = clf.predict(image, confidence = 40, overlap = 50)
    annotated_frame, label, x = processing(image, outputs.json()['predictions'])
    height, width = annotated_frame.shape[:2]
    try:
        pub_label.publish(label[0])
        pub_align.publish(640 - int(x[0]))
        # pub_align.publish(width)
    except Exception as e:
        pass
    cv2.imshow('Realsense Stream', annotated_frame)
    cv2.waitKey(1)


    
if __name__ == '__main__':
    # model_dir = '/home/aman/catkin_ws/src/blackflag/realsense_stream/config'
    # model_name = 'yolov8_v2.pth'
    # model_path = os.path.join(model_dir, model_name)
    # clf = torch.load(model_path)
    rf = Roboflow(api_key="uKIsEXbOt27HBKcS5KAo")
    project = rf.workspace().project("mechatronics2")
    clf = project.version(1).model

    rospy.init_node('image_predict', anonymous=True)
    rospy.Subscriber('/camera/color/image_raw', Image, callback)
    pub_label = rospy.Publisher('/realsense_predict', String, queue_size = 10)
    pub_align = rospy.Publisher('/align_factor', Int32, queue_size = 10)
    rospy.spin()
