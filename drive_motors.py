#!/usr/bin/env python
from __future__ import division

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist


class Motor_Driver:
    def __init__(self):
        self.gain = 1/2 
        self.basespeed = 10

        rospy.init_node('motor_driver', anonymous=True)
        rospy.Subscriber("/rrbot/cv_stear_error", Int16, self.callback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
            

    def callback(self, data):
        rospy.loginfo(data.data+self.gain)
        s = Twist()
        s.linear.x = self.basespeed
        s.angular.z = data.data*self.gain
        self.pub.publish(s)
     

if __name__ == '__main__':
    driver = Motor_Driver()
    rospy.spin()