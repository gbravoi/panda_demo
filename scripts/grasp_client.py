#!/usr/bin/env python2

import sys
import rospy
from panda_demo.srv import *

def compute_grasps_client(object_name):
    #wait for service to be available
    rospy.wait_for_service('ComputeGrasps')

    #try to use service
    try:
        compute_grasp_service = rospy.ServiceProxy('ComputeGrasps', computegraps)
        resp1 = compute_grasp_service(object_name)
        return resp1.grasp_result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s wrong argument, it should be [object_name]"%sys.argv[0]

if __name__ == "__main__":
    #ask object name as parameter of the client 
    if len(sys.argv) == 2:
        object_name = str(sys.argv[1])
    else:
        print usage()
        sys.exit(1)
    
    #save computed grasp in variable g
    g=compute_grasps_client(object_name)
    #print "Returning grasp [%s , %s , %s]"%(g.grasp_pose.pose.position.x, g.grasp_pose.pose.position.y, g.grasp_pose.pose.position.z)

