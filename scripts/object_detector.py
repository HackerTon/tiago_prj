#!/usr/bin/env python

import sys

import cv_bridge
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from tiago_prj.msg import Message

import cv2
import tensorflow


class Detector:
    def __init__(self, argv):
        rospy.loginfo('Waiting for image_raw from tiago camera')
        rospy.wait_for_message(
            'xtion/rgb/image_raw', Image
        )

        # subscribe to image topic
        self.img_sub = rospy.Subscriber(
            'xtion/rgb/image_raw', Image, self.callback
        )

        # publish to message
        self.img_sub = rospy.Publisher(
            '/bounding_box', Message, queue_size=1
        )

        self.bridge = CvBridge()
        rospy.loginfo('NODE: ' + rospy.get_name() + ' ready!')

    def callback(self, img_data):
        cv_image = None

        # convert Image to cv2 Mat format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_data)
            
        except CvBridgeError as e:
            print('#'*30 + 'ERROR at cvbridge' + '#'*30)
            print(e.message)

        # debug show img
        if cv_image is not None:
            # show img in window
            cv2.imshow('image', cv_image)
            cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('object_detector')

    Detector(sys.argv)
    rospy.spin()
