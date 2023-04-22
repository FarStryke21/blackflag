#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
<<<<<<< HEAD
from std_msgs.msg import String, Int32
=======
from std_msgs.msg import String
>>>>>>> 4f5f9ed3e7934c127a749719ca23aff68ab53dd0
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
<<<<<<< HEAD
    xs = []
=======
>>>>>>> 4f5f9ed3e7934c127a749719ca23aff68ab53dd0
    try :
        for output in outputs:
            x = output['x']
            y = output['y']
<<<<<<< HEAD
            xs.append(x)
=======
>>>>>>> 4f5f9ed3e7934c127a749719ca23aff68ab53dd0
            w = output['width']
            h = output['height']
            return_frame = cv2.rectangle(return_frame, (int(x - w/2), int(y - h/2)), (int(x + w/2), int(y + h/2)), (0,0,255), 2)
            label = output['class']
            labels.append(label)
            return_frame = cv2.putText(return_frame, label, (int(x - w/2), int(y - h/2) - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
    except Exception:
        pass

<<<<<<< HEAD
    return return_frame, labels, xs
=======
    return return_frame, labels
>>>>>>> 4f5f9ed3e7934c127a749719ca23aff68ab53dd0

def prediction(image):
    # frame = np.asanyarray(image)
    outputs = clf.predict(image, confidence = 40, overlap = 50)
<<<<<<< HEAD
    annotated_frame, label, x = processing(image, outputs.json()['predictions'])
    height, width = annotated_frame.shape[:2]
    try:
        pub_label.publish(label[0])
        pub_align.publish(640 - int(x[0]))
        # pub_align.publish(width)
=======
    annotated_frame, labels = processing(image, outputs.json()['predictions'])
    try:
         pub.publish(labels[0])
>>>>>>> 4f5f9ed3e7934c127a749719ca23aff68ab53dd0
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
<<<<<<< HEAD
    pub_label = rospy.Publisher('/realsense_predict', String, queue_size = 10)
    pub_align = rospy.Publisher('/align_factor', Int32, queue_size = 10)
=======
    pub = rospy.Publisher('/realsense_predict', String, queue_size = 10)
>>>>>>> 4f5f9ed3e7934c127a749719ca23aff68ab53dd0
    rospy.spin()
