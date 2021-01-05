#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time
from sensor_msgs.msg import Image
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import message_filters

# cv_color_image = []
# cv_depth_image = []
# topic_list = ['/camera/color/image_raw', '/camera/depth/image_rect_raw']
# topic_list_map = {"color" : 0, "depth" : 1}
# bridge = CvBridge()

# def ImageColorCallBack(data):
#     try:
#         print 1
#         cv_color_image = bridge.imgmsg_to_cv2(data, data.encoding)
#         pix = (data.width/2, data.height/2)
#         # sys.stdout.write('%s: color at center(%d, %d): %s \r' % (self.topic_list[self.topic_list_map["color"]], pix[0], pix[1], cv_color_image[pix[1], pix[0]]))
#         print '%s: color at center(%d, %d): %s \r' % (topic_list[topic_list_map["color"]], pix[0], pix[1], cv_color_image[pix[1], pix[0]])
#         # for i in range(0, 12):
#         #     for j in range(0, 16):
#         #         sys.stdout.write('%s ' % (cv_image[i*40, j*40]))
#         #         sys.stdout.write('%d %d ' % (i, j))
#         #     sys.stdout.write('\n')
#         # time.sleep(10)
#         # time.sleep(1)
#         # sys.stdout.flush()
#     except CvBridgeError as e:
#         print(e)
#         return

# def ImageDepthCallBack(data):
#     try:
#         print 2
#         cv_depth_image = bridge.imgmsg_to_cv2(data, data.encoding)
#         pix = (data.width/2, data.height/2)
#         # sys.stdout.write('%s: Depth at center(%d, %d): %f(mm)\r' % (self.self.topic_list[self.topic_list_map["depth"]], pix[0], pix[1], cv_depth_image[pix[1], pix[0]]))
#         print '%s: Depth at center(%d, %d): %f(mm)\r' % (topic, pix[0], pix[1], cv_depth_image[pix[1], pix[0]])
#         # for i in range(0, 12):
#         #     for j in range(0, 16):
#         #         sys.stdout.write('%f ' % (cv_image[i*40, j*40]))
#         #         sys.stdout.write('%d %d ' % (i, j))
#         #     sys.stdout.write('\n')
#         # time.sleep(10)
#         # time.sleep(1)
#         # sys.stdout.flush()
#     except CvBridgeError as e:
#         print(e)
#         return

# depth_data = rospy.Subscriber('/camera/depth/image_raw', Image, ImageDepthCallBack, queue_size=1)
# print 3
# color_data = rospy.Subscriber('/camera/color/image_raw', Image, ImageColorCallBack, queue_size=1)
# print 4

# def multi_callback(Subcriber_laser,Subcriber_pose):
#     print "同步完成！"

class MySys:
    def __init__(self):
        self.cv_color_image = []
        self.cv_depth_image = []
        self.topic_list = ['/camera/color/image_raw', '/camera/depth/image_rect_raw']
        self.topic_list_map = {"color" : 0, "depth" : 1}
        self.listener1 = GetColorImageInfo(self)
        self.listener2 = GetDepthImageInfo(self)
        # self.listener2 = GetDepthImageInfo()
        self.showflag = True
        self.targetpose = []
        
    def show(self):
        while self.showflag:
            print 'Color at center(320, 240): %s \r' % self.cv_color_image[240, 320]
            print 'Depth at center(320, 240): %s \r' % self.cv_depth_image[240, 320]
            time.sleep(1)

    def Loop(self):
        print "Starting Loop..."
        rospy.spin()

class GetDepthImageInfo(object):
    def __init__(self, mysys):
        self.sys = mysys
        self.topic = '/camera/depth/image_rect_raw'
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(self.topic, Image, self.ImageCallBack)
# class GetDepthImageInfo(object):
#     def __init__(self):
#         self.topic = '/camera/depth/image_rect_raw'
#         self.bridge = CvBridge()
#         self.sub = rospy.Subscriber(self.topic, Image, self.ImageCallBack)

    def ImageCallBack(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.sys.cv_depth_image = cv_image
            pix = (data.width/2, data.height/2)
            sys.stdout.write('%s: Depth at center(%d, %d): %f(mm)\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))
            # print '%s: Depth at center(%d, %d): %f(mm)\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]])
            # for i in range(0, 12):
            #     for j in range(0, 16):
            #         sys.stdout.write('%f ' % (cv_image[i*40, j*40]))
            #         # sys.stdout.write('%d %d ' % (i, j))
            #     sys.stdout.write('\n')
            # time.sleep(10)
            sys.stdout.flush()
        except CvBridgeError as e:
            print(e)
            return
# target_pose.pose.orientation.x = -0.611
#         target_pose.pose.orientation.y = 0.313
#         target_pose.pose.orientation.z = -0.338
#         target_pose.pose.orientation.w = 0.644
class GetColorImageInfo(object):
    def __init__(self, mysys):
        self.sys = mysys
        self.topic = '/camera/color/image_raw'
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(self.topic, Image, self.ImageCallBack)
    
    def ImageCallBack(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.sys.cv_color_image = cv_image
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

class targetlocation():
    def __init__(self, mysys):
        pass

class urmove():
    def __init__(self, mysys):
        self.oldpose = []
        self.newpose = mysys.targetpose

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    # listener1 = MySys()
    # sync = message_filters.ApproximateTimeSynchronizer([depth_data, color_data],10,0.1,allow_headerless=True)
    # sync.registerCallback(multi_callback)
    print "Starting Loop..."
    gyh = MySys()
    rospy.spin()
    # listener.Loop()