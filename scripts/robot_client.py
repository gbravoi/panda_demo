#!/usr/bin/env python2

#example of client for the robot, used to test robot server when complex message neded

import sys
import rospy
from panda_demo.srv import *
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler
from math import pi

#------------------------------------
#--------CLIENT FUNCTIONS---------
#------------------------------------


def goPose_client(pose_goal):
    #wait for service to be available
    rospy.wait_for_service('RobotCommand/RequestPose')

    #try to use service
    try:
        go_pose_service = rospy.ServiceProxy('RobotCommand/RequestPose', robotRequestPose)
        resp1 = go_pose_service(pose_goal)
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def PickPlace_client(object_name,place_pose):
    #wait for service to be available
    rospy.wait_for_service('RobotCommand/PickPlace')

    #try to use service
    try:
        pickplace_service = rospy.ServiceProxy('RobotCommand/PickPlace', robotPickPlace)
        resp1 = pickplace_service(object_name,place_pose)
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



if __name__ == "__main__":
    #ask object name as parameter of the client 
    command_name = str(sys.argv[1])
    
    if command_name =="Pose":
        #example pose
        pose_goal = Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4
        result=goPose_client(pose_goal)
        print result

    elif command_name =="Pick":
        #place pose
        #define pose where to place object
        place_pose = Pose()
        place_pose.position.x = 0 
        place_pose.position.y = 0.5
        place_pose.position.z = 0.5
        #orientation in quaternion
        q = quaternion_from_euler(0,0,pi/2)
        place_pose.orientation.x = q[0]
        place_pose.orientation.y = q[1]
        place_pose.orientation.z = q[2]
        place_pose.orientation.w = q[3]
        
        object_name="block"
        result=PickPlace_client(object_name,place_pose)
        #call service
        print result


   
