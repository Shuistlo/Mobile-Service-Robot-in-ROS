# !/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

#this code a modified version of The Construct's tutorial, available at
#http://www.theconstructsim.com/ros-qa-140-how-to-modify-a-robots-coordinates-when-it-arrives-at-a-checkpoint/

#modifies a robot's perceived coordinates by publishing new coordinates
class poseSetter:
    def __init__(self):
        #rospy.init_node('the_only_node', anonymous=True)
        self.pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        rospy.sleep(2)
        self.newPos = PoseWithCovarianceStamped()

        self.newPos.pose.pose.position.x = 0.0
        self.newPos.pose.pose.position.y = 0.0
        self.newPos.pose.pose.position.z = 0.0

        [x, y, z, w] = quaternion_from_euler(0.0, 0.0, 0.0)
        self.newPos.pose.pose.orientation.x = x
        self.newPos.pose.pose.orientation.y = y
        self.newPos.pose.pose.orientation.z = z
        self.newPos.pose.pose.orientation.w = w

#modifies a robot's perceived coordinates by publishing new coordinates
    def newPose(self, nx, ny):
        self.newPos.pose.pose.position.x = nx
        self.newPos.pose.pose.position.y = ny

        self.pub.publish(self.newPos)
'''
if __name__ == "__main__":
    poser = poseSetter()
    poser.newPose(33.4, 20.5)
'''
