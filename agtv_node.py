#!/usr/bin/env python
# cartesian joystick waypoint control for quadrotor
#import roslib
#roslib.load_manifest('quad_control')
#import rospy
import copy
import math
import serial
import numpy as np
#from geometry_msgs.msg import PoseStamped, PointStamped, Pose

# local imports
#from acl_system.msg import ViconState


class AGTV_CONTROL:
    def __init__(self):
        self.pose = np.zeros(3) # [x y z]
        self.quat  = np.zeros(4) # [qw qx qy qz]
        self.omega = np.zeros(3) # [wx wy wz]
        

    def viconCB(self, data):      
        self.pose = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.quat = np.array([data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z])
        self.omega = np.array([data.twist.linear.x, data.twist.linear.y, data.twist.linear.z])

        #self.agtvController()     
    def test(self, word):
        print word + " world"



##def startNode():
##    rospy.init_node('agtv')
##    c = AGTV_CONTROL()
##    rospy.Subscriber("/AGTV/vicon", ViconState, c.viconCB)
##    rospy.loginfo("AGTV node initialized")
##    rospy.spin()
##
##if __name__ == '__main__':
##    startNode()

