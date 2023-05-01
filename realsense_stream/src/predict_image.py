#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
# from cv_bridge import CvBridge
from PIL import Image as PILImage
import os
import numpy as np
from roboflow import Roboflow
import roboflow
import panel_classifier
# import yolo_classifier

def callback(msg):
    pil_image = PILImage.frombytes("RGB",(msg.width, msg.height), msg.data)
    cv_image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
    prediction(cv_image)

def processing(frame, outputs):
    return_frame = frame
    labels = []
    xs = []
    cropped_frame = []
    try :
        for output in outputs:
            x = output['x']
            y = output['y']
            xs.append(x)
            w = output['width']
            h = output['height']
            return_frame = cv2.rectangle(return_frame, (int(x - w/2), int(y - h/2)), (int(x + w/2), int(y + h/2)), (0,0,255), 2)
            cropped_frame.append(frame[int(y - h/2):int(y + h/2), int(x - w/2):int(x + w/2)])
            label = output['class']
            labels.append(label)
            return_frame = cv2.putText(return_frame, label, (int(x - w/2), int(y - h/2) - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
    except Exception:
        pass

    return return_frame, labels, xs, cropped_frame

def prediction(image):
    outputs = clf.predict(image, confidence = 30, overlap = 30)
    annotated_frame, label, x, cropped_frames = processing(image, outputs.json()['predictions'])
    # height, width = annotated_frame.shape[:2]
    try:
        # for i in range(len(label)):
        if label[0] == 'horizontal stopcock':
            status = panel_classifier.hor_stpck(cropped_frames[0])
            label[0] = label[0]+':'+status
        elif label[0] == 'vertical stopcock':
            status = panel_classifier.ver_stpck(cropped_frames[0])
            label[0] = label[0]+':'+status
        elif label[0] == 'breaker':
            pass
        # message = ";".join(label)
        # pub_label.publish(message)
        pub_label.publish(label[0])
        # pub_align.publish(640 - int(x[0]))
        # pub_align.publish(width)

    except Exception as e:
        # rospy.loginfo(e)
        pass

    cv2.imshow('Realsense Stream', annotated_frame)
    cv2.waitKey(1)


    
if __name__ == '__main__':
    rf = Roboflow(api_key="uKIsEXbOt27HBKcS5KAo")
    project = rf.workspace().project("mechatronics2")
    clf = project.version(1).model
    print("Model Loaded")
    rospy.loginfo("Model loaded, beginning inference ...")

    rospy.init_node('image_predict', anonymous=True)
    rospy.Subscriber('/camera/color/image_raw', Image, callback)
    pub_label = rospy.Publisher('/realsense_predict', String, queue_size = 10)
    # pub_align = rospy.Publisher('/align_factor', Int32, queue_size = 10)
    # pub_cropped = rospy.Publisher('/cropped_view', Image, queue_size = 10)
    rospy.spin()
