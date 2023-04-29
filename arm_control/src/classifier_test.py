#!/usr/bin/env python

import cv2
import numpy as np
import math
import rospy
from std_msgs.msg import String
import panel_classifier

if __name__ == '__main__':
    file = '/home/aman/video2.avi'

    cap = cv2.VideoCapture(file)

    # Get the size of the video frame
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Print the size of the video frame
    print('Frame size: {}x{}'.format(frame_width, frame_height))

    # Loop through the frames of the video
    while cap.isOpened():
        # Read a frame from the video
        ret, frame = cap.read()

        # If the frame was read successfully
        if ret:
            # Display the frame in a window titled "Video"
            status = panel_classifier.hor_stpck(frame)

            print(status)
            cv2.imshow('Video', frame)
            # Wait for 25 milliseconds and check if the user pressed the "q" key
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break
        else:
            break

    # Release the video capture object and close all windows
    cap.release()
    cv2.destroyAllWindows()
