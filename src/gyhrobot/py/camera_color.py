#! /usr/bin/env python

import time
from sensor_msgs.msg import Image
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys
import os

class GetColorImageInfo:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, Image, self.ImageCallBack)
    
    def ImageCallBack(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            pix = (data.width/2, data.height/2)
            sys.stdout.write('%s: color at center(%d, %d): %s \r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))
            # print '%s: Depth at center(%d, %d): %f(mm)\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]])
            # for i in range(0, 12):
            #     for j in range(0, 16):
            #         sys.stdout.write('%s ' % (cv_image[i*40, j*40]))
            #         # sys.stdout.write('%d %d ' % (i, j))
            #     sys.stdout.write('\n')
            # time.sleep(10)
            sys.stdout.flush()
        except CvBridgeError as e:
            print(e)
            return

    def Show(self, cv_DepthData):
        # assert isinstance(DepthData, Image)
        # depth = self.bridge.imgmsg_to_cv2(DepthData, DepthData.encoding)
        cv2.imshow('depth', cv_DepthData)
        cv2.waitKey(0)
        time.sleep(1)
        return

def main():
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    topic = '/camera/color/image_raw'
    listener = GetColorImageInfo(topic)
    rospy.spin()

if __name__ == "__main__":
    main()
