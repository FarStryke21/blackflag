#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def camera_publisher():
    # initialise the node
    rospy.init_node('endoscope_camera', anonymous=True)

    rate = rospy.Rate(30)

    image_pub = rospy.Publisher('/camera/endoscope_stream', Image, queue_size=10)

    bridge = CvBridge()

    # Open the default camera (TUNE THIS FOR NANO PORTS)
    cap = cv2.VideoCapture(2)

    # Set the resolution of the camera
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    while not rospy.is_shutdown():
        # Read a frame from the camera
        ret, frame = cap.read()

        # Check if the frame was successfully read
        if not ret:
            print("Error reading frame from camera")
            break
        else:
            ros_image = bridge.cv2_to_imgmsg(frame, encoding = 'bgr8')
            image_pub.publish(ros_image)
        rate.sleep()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass