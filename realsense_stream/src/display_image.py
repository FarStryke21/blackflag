#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import String
from cv_bridge import CvBridge

def callback(image_msg):
    # Convert ROS image
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_msg, desired_encoding = 'passthrough')
    cv_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    cv2.imshow('Image', cv_image)
    cv2.waitKey(1)

def main():
    rospy.init_node('image_display', anonymous=True)
    rospy.Subscriber('/camera/color/image_raw', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    main()