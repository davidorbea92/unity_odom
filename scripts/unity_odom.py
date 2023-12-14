#!/usr/bin/env python3


import rospy
from nav_msgs.msg import Odometry

rosRate=1
class OdometryUnity:
  def __init__(self):
    self.odometry_publisher = rospy.Publisher("/odom_low", Odometry, queue_size=1)
    self.odom_pose=Odometry()

  def odometryCb(self, msg):
    self.odom_pose=msg
    
  def publish_odom(self,event=None):
    self.odom_msg=Odometry()
    self.odom_msg=self.odom_pose
    self.odometry_publisher.publish(self.odom_msg)
      
  def start(self):
    rospy.Subscriber('/odom',Odometry,self.odometryCb)
    rospy.Timer(rospy.Duration(1.0/rosRate), self.publish_odom)
    rospy.spin()

  
    

if __name__ == '__main__':
  rospy.init_node('odometry_unity', anonymous=True) #make node
  odom1=OdometryUnity()
  try:
    odom1.start()
  except rospy.ROSInterruptException:
    pass