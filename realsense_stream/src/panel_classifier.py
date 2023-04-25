#!/usr/bin/env python3

import cv2
import numpy as np
import math
import rospy
from std_msgs.msg import String


def ver_stpck(img):
    # April 24th: Working for live stream
    # x1, y1 = 300, 200
    # x2, y2 = 1000, 700
    # img = img[y1:y2, x1:x2]
    # Convert to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the range of blue color in HSV
    lower_blue = np.array([100, 150, 100])
    upper_blue = np.array([120, 255, 255])
    try:
        # Create a mask for the blue color
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        kernel1 = np.ones((3,3),np.uint8)
        mask = cv2.erode(mask,kernel1,iterations = 6)

        kernel2 = np.ones((5,5),np.uint8)
        mask = cv2.dilate(mask,kernel2,iterations = 10)
        
        # Apply a threshold to the grayscale mask image to obtain a binary image
        thresh = cv2.threshold(mask, 0, 255, cv2.THRESH_BINARY)[1]

        # # Find the contours in the image
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # thresh = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        # thresh = cv2.drawContours(thresh, contours, -1, (0, 0, 255), 5)
        # print(contours)
        # # Find the contour with the largest area, which should correspond to the blue rectangle
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        # cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 5)
        # # # Fit an ellipse to the contour
        ellipse = cv2.fitEllipse(largest_contour)

        # # Get the parameters of the ellipse
        center, axes, angle = ellipse

        # # Draw the major axis of the ellipse on the original image
        # major_axis = (int(center[0]), int(center[1])), (int(axes[0]/2), 0), angle
        # minor_axis = (int(center[0]), int(center[1])), (0, int(axes[1]/2)), angle
        # cv2.ellipse(img, minor_axis, (0, 0, 255), 5)

        # cv2.line(thresh, major_axis[0], major_axis[1], (0, 0, 255), 5)
        # cv2.line(thresh, minor_axis[0], minor_axis[1], (0, 0, 255), 5)
        angle = abs(angle-90)
        status = '?'
        if angle < 10:
            status = 'close'
        else:
            status = 'open'


        # cv2.putText(img, status, (x, y-40), cv2.FONT_HERSHEY_SIMPLEX, 5, (0, 0, 255), 10)
        # display(img)
        return status
    except Exception as e:
        return '?'

def hor_stpck(img):
    # x1, y1 = 200, 300
    # x2, y2 = 1200, 700
    # img = img[y1:y2, x1:x2]
    # Convert to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the range of blue color in HSV
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([120, 255, 255])
    try:
        # Create a mask for the blue color
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        kernel1 = np.ones((3,3),np.uint8)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        # Perform dilation and erosion on the mask
        mask = cv2.erode(mask,kernel1,iterations = 6)

        kernel2 = np.ones((5,5),np.uint8)
        mask = cv2.dilate(mask,kernel2,iterations = 6)
        
        # Apply a threshold to the grayscale mask image to obtain a binary image
        thresh = cv2.threshold(mask, 0, 255, cv2.THRESH_BINARY)[1]

        # # Find the contours in the image
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        thresh = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

        # # Find the contour with the largest area, which should correspond to the blue rectangle
        largest_contour = max(contours, key=cv2.contourArea)
        # x, y, w, h = cv2.boundingRect(largest_contour)
        # cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 5)
        # # # Fit an ellipse to the contour
        ellipse = cv2.fitEllipse(largest_contour)
        # Draw the ellipse on a blank image
        img_ellipse = np.zeros_like(img)
        cv2.ellipse(img_ellipse, ellipse, (255, 255, 255), 2)

        # Measure the length of the major axis
        major_axis_length = max(ellipse[1])

        # # Get the parameters of the ellipse
        center, axes, angle = ellipse

        # # Draw the major axis of the ellipse on the original image
        # major_axis = (int(center[0]), int(center[1])), (int(axes[0]/2), 0), angle
        # minor_axis = (int(center[0]), int(center[1])), (0, int(axes[1]/2)), angle
        # cv2.ellipse(img, minor_axis, (0, 0, 255), 5)

        angle = abs(angle-90)
        status = '?'
        if major_axis_length < 90:
            status = 'close'
        else:
            status = 'open'

        return status
    except Exception as e:
        return '?'

def brk(img):
    x1, y1 = 750,1000
    x2, y2 = 2500, 2500
    img = img[y1:y2, x1:x2]
    # -------------------------------------CENTERLINE OF THE BREAKERS--------------------------------------------------------
    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Threshold the grayscale image to create a binary mask for black regions
    _, mask = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)

    kernel2 = np.ones((3,3),np.uint8)
    mask = cv2.dilate(mask,kernel2,iterations = 1)
    
    kernel1 = np.ones((5,5),np.uint8)
    mask = cv2.erode(mask,kernel1,iterations = 3)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    second_level_contours = []
    for i, contour in enumerate(contours):
        if hierarchy[0][i][3] != -1 and hierarchy[0][hierarchy[0][i][3]][3] == -1:
            second_level_contours.append(contour)

    # Sort the second level contours by area (largest to smallest)
    second_level_contours = sorted(second_level_contours, key=cv2.contourArea, reverse=True)


    # Draw contours on the original image
    breaker_center = []
    output = img.copy()
    for i in range(3):
        if i < len(second_level_contours):
            ellipse = cv2.fitEllipse(second_level_contours[i])
            center, axes, angle = ellipse
            breaker_center.append(center)
            major_axis = (int(center[0]), int(center[1])), (int(axes[0]/2), 0), angle
            minor_axis = (int(center[0]), int(center[1])), (0, int(axes[1]/2)), angle
            cv2.ellipse(output, major_axis, (0, 0, 255), 5)

    breaker_center = sorted(breaker_center, key=lambda x: x[0])

    # -------------------------------------CENTERLINE OF THE SWITCHES--------------------------------------------------------
    # Convert the image to the HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for the orange color
    lower_orange = np.array([10, 150, 140])
    upper_orange = np.array([15, 255, 255])

    # Create a mask for the orange color
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    kernel1 = np.ones((3,3),np.uint8)
    mask = cv2.erode(mask,kernel1,iterations = 1)

    kernel2 = np.ones((11,11),np.uint8)
    mask = cv2.dilate(mask,kernel2,iterations = 5)
    
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    switch_center = []
    for i, contour in enumerate(contours):
        ellipse = cv2.fitEllipse(contour)
        center, axes, angle = ellipse
        switch_center.append(center)
        major_axis = (int(center[0]), int(center[1])), (int(axes[0]/2), 0), angle
        minor_axis = (int(center[0]), int(center[1])), (0, int(axes[1]/2)), angle
        cv2.ellipse(output, minor_axis, (0, 255, 0), 5)
        
    switch_center = sorted(switch_center, key=lambda x: x[0])

    distances = []
    for i in range(0, len(switch_center)):
        point1 = switch_center[i]
        point2 = breaker_center[i]
        distances.append(math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2))

    print(distances)

    status = []
    for i in range(0, len(distances)):
        if distances[i] > 100:
            status.append("1")
        else:
            status.append("-1")

    status = ' '.join(status)
    # cv2.putText(output, status, (200, 800), cv2.FONT_HERSHEY_SIMPLEX, 5, (0, 0, 255), 10)
    return status