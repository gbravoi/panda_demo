#!/usr/bin/env python2

#The idea of this node is to move the robot to perform differnet task
#using simple comands
import sys
import rospy
from panda_demo.srv import *
import RobotCommander #phyton file with all the fucntions of the robot.


#------------------------------------
#--------AUXILIARY FUNCTIONS---------
#------------------------------------
def compute_grasps_client(object_name):
    #function to get grasp for an object using ComputeGrasps Service
    #Current way is working is giving only one grasp.
    #in the future, with more complex algorithms, you can call several times to
    #get different grasp poses?
    #wait for service to be available
    rospy.wait_for_service('ComputeGrasps')

    #try to use service
    try:
        compute_grasp_service = rospy.ServiceProxy('ComputeGrasps', computegraps)
        resp1 = compute_grasp_service(object_name)
        return resp1.grasp_result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def openGripperPose(posture):
    #auxiliary function needed for pick-place
    posture.joint_names=[]
    posture.joint_names.append("panda_finger_joint1")
    posture.joint_names.append("panda_finger_joint2")

    #Set them as open. 
    pos = JointTrajectoryPoint()
    pos.positions.append(0.04) #pos finger 1 (in maximum position)
    pos.positions.append(0.04) #pos finger 2
    pos.time_from_start=rospy.Duration(0.5)
    posture.points.append(pos)

#------------------------------------
#--------CALLBACK FUNCTION---------
#------------------------------------
def set_pose(req):
    #move the robot to a asked pose
    pose=req.pose_goal
    Panda_R.go_to_pose_goal(pose)
    return True

def simple_command(req):
    command=req.command

    if command=="OpenGripper":
        Panda_R.openGripper()
    elif command=="CloseGripper":
        Panda_R.closeGripper()
    elif command=="Home":
        Panda_R.go_home()

    return True


def pick_place(req):
    object_name=req.object_name
    pose=req.place_pose

    Panda_R.pick(object_name)
    Panda_R.place(pose)

    return True


    
#------------------------------------
#--------SERVER CREATION---------
#------------------------------------
def robot_server():
    #init server node
    rospy.init_node('robot_server')

    #this node handle multiple services
    #request a specific pose to the robot
    s1 = rospy.Service('RobotCommand/RequestPose', robotRequestPose, set_pose) #service name, type, callback function

    #simple commands: OpenGripper, CloseGripper, Home
    s2 = rospy.Service('RobotCommand/SimpleCommand', robotSimpleCommand, simple_command) #service name, type, callback function

    #pick an place object
    s3 = rospy.Service('RobotCommand/PickPlace', robotPickPlace, pick_place) #service name, type, callback function

    #print "Ready to compute the grasp for an object"
    #keep running the service
    rospy.spin()


#------------------------------------
#--------MAIN---------
#------------------------------------
if __name__ == "__main__":
    
    #start robot
    Panda_R = RobotCommander.Robot()

    #initialize services
    robot_server()

    print "Ready for service"