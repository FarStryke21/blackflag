#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import String
from cv_bridge import CvBridge
import os
import torch

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
    for output in outputs:
        x = output['x']
        y = output['y']
        w = output['width']
        h = output['height']
        return_frame = cv2.rectangle(return_frame, int(x - w/2), int(y - h/2), int(x + w/2), int(y + h/2), (0,0,255), 2)
        label = output['class']
        labels.append(label)
        return_frame = cv2.putText(return_frame, label, (int(x - w/2), int(y - h/2) - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
    return return_frame, labels

def prediction(image):
    outputs = clf.predict(image, confidence = 1, overlap = 50)
    annotated_frame, labels = processing(image, outputs.prediction)
    
    try:
        publisher(labels)
    except rospy.ROSInterruptException:
        pass

    cv2.imshow('Realsense Stream', annotated_frame)
    cv2.waitKey(1)

def publisher(labels):
    # rospy.init_node('image_predict', anonymous=True)
    pub = rospy.Publisher('/realsense_predict', String, queue_size = 10)
    pub.publish(labels[0])
    
def main():
    rospy.init_node('image_predict', anonymous=True)
    rospy.Subscriber('/camera/color/image_raw', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    model_dir = '/home/mechatronics/catkin_ws/src/blackflag/realsense_stream/config'
    model_name = 'yolov8_v2.pth'
    model_path = os.path.join(model_dir, model_name)

    clf = torch.load(model_path)

    main()