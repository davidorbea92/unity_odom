#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist, TwistStamped

rosRate=30
class jaguar_twist:
  def __init__(self):
    self.twist_publisher = rospy.Publisher("/jaguar/cmd_vel", TwistStamped, queue_size=1)
    self.twist_msg=Twist()

  def twistCb(self, msg):
    self.twist_msg=msg
    
  def publish_twist(self,event=None):
    self.twist_stamped_msg=TwistStamped()
    #self.twist_stamped_msg.header=""
    self.twist_stamped_msg.twist=self.twist_msg
    self.twist_publisher.publish(self.twist_stamped_msg)
      
  def start(self):
    rospy.Subscriber('cmd_vel',Twist,self.twistCb)
    rospy.Timer(rospy.Duration(1.0/rosRate), self.publish_twist)
    rospy.spin()

if __name__ == '__main__':
  rospy.init_node('jaguar_twist', anonymous=True) #make node
  twist1=jaguar_twist()
  try:
    twist1.start()
  except rospy.ROSInterruptException:
    pass