#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter(object):
    def __init__(self, topic_name, file_name, fps=20):
        self.fps = fps
        self.file_name = file_name
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic_name, Image, self.callback)
        self.video_writer = None

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.loginfo(str(e))

        if self.video_writer is None:
            rows, cols, _ = cv_image.shape
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            self.video_writer = cv2.VideoWriter(self.file_name, fourcc, self.fps, (cols, rows))

        self.video_writer.write(cv_image)

    def clean_shutdown(self):
        if self.video_writer is not None:
            self.video_writer.release()

    def __del__(self):
        self.clean_shutdown()

def main():
    ic = image_converter('/camera/color/image_raw','video.avi')
    rospy.init_node('test_video', anonymous=True)
    rospy.on_shutdown(ic.clean_shutdown)
    rospy.spin()

if __name__ == '__main__':
    main()