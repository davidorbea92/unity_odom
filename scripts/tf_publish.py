#!/usr/bin/env python3

import rospy, math
import numpy as np
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8
import math
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf

rate=20

class tf_publisher(object):
    def __init__(self):
        self.broadcaster=tf.TransformBroadcaster()
        self.pose=Pose()
        self.pose.orientation.w=1.0

    def pose_callback(self,msg):
        self.pose=msg

    def tf_timer(self,event=None):
        self.broadcaster.sendTransform((self.pose.position.x, self.pose.position.y, 0.343),
                                           (self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w),
                                           rospy.Time.now(),
                                           "world",
                                           "map")
        #self.broadcaster.sendTransform((0, 0, 0),
        #                                   tf.transformations.quaternion_from_euler(0, 0, 3.1416),
        #                                   rospy.Time.now(),
        #                                   "world_turn",
        #                                   "world")

    def start(self):
        rospy.Subscriber('/robot/pose', Pose, self.pose_callback)
        rospy.Timer(rospy.Duration(1/rate), self.tf_timer)

        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('tf_publish')
    main=tf_publisher()
    try:
        main.start()
    except rospy.ROSInterruptException:
        pass