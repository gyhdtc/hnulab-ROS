#! /usr/bin/env python

from sensor_msgs.msg import PointCloud2, CameraInfo, Image
from sensor_msgs import point_cloud2
import rospy
import time

def callback(data):
    assert isinstance(data, PointCloud2)
    gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    
    print type(gen)
    i = 0
    for p in gen:
      print " x : %.3f  y: %.3f  z: %.3f [%d]" %(p[0],p[1],p[2],i)
      i = i + 1
    time.sleep(5)
def main():
    rospy.init_node('get_pcl', anonymous=True)
    rospy.Subscriber('/camera/depth_registered/points', PointCloud2, callback)
    rospy.spin()

if __name__ == "__main__":
    main()