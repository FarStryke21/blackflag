#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Define the callback function to handle the image data
def image_callback(msg):
    # Convert the ROS Image message to a numpy array using cv_bridge
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding = 'passthrough')
    cv_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    # Write the current frame to the video file
    cv2.imshow('Image', cv_image)
    cv2.waitKey(1)

    out.write(cv_image)


# Create a video writer to save the frames as a video file
out = cv2.VideoWriter('/home/aman/catkin_ws/src/blackflag/realsense_stream/videos/realsense_video.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 30, (640, 480))

# Initialize the node and subscriber
rospy.init_node('realsense_video')
rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
rospy.loginfo("Stream Started!")
# Start the video writer and keep the node running
while not rospy.is_shutdown():
    rospy.spin()
    if cv2.waitKey(1) & 0xFF == ord('s'):
            break

# Release the video writer when the node is shut down
out.release()
rospy.loginfo("Video Saved!")