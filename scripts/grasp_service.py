#!/usr/bin/env python2

#This is a service that only work in the example
#in the future replace by a service that can compute the grasp of an object

#costum service message computegrasp.srv has been created

from panda_demo.srv import computegraps,computegrapsResponse
import rospy
from moveit_msgs.msg import  Grasp
from geometry_msgs.msg import PoseStamped, Pose
from gazebo_msgs.srv import GetModelState #gazebo service
from tf.transformations import quaternion_from_euler
from math import pi
from trajectory_msgs.msg import JointTrajectoryPoint

#------------------------------------
#--------AUXILIARY FUNCTIONS---------
#------------------------------------

def Query_object_Pose( object_name):
    #query pose to the scene from rviz need moveit scene
    # pos_list=scene.get_object_poses([object_name])
    # pos=Pose()
    # pos.position= pos_list[object_name].position
    # pos.orientation= pos_list[object_name].orientation
    
    #query from gazebo
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        #call service that return model state
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp1 = gms(model_name=object_name)
    except rospy.ServiceException, e:
        rospy.logerr('Error on calling service: %s',str(e))
        print "Service call failed: %s"%e     
    print resp1.pose

    # #manual pose
    # box_pose=Pose()
    # box_pose.position.x=0.4225 
    # box_pose.position.y=0.1265
    # box_pose.position.z=0.1213
    #return box_pose #manual pose
    return resp1.pose #gazebo pose

#def Query_object_dimensions(object_name):
    #return object dimensions rviz, need moveit scene
    # obj_list=scene.get_objects([object_name])
    # print "object width : %s" %obj_list[object_name].primitives[0].dimensions[0]
    # return obj_list[object_name].primitives[0].dimensions

def closeGripperPose(posture,object_width):
    #auxyliary function needed for pick
    posture.joint_names=[]
    posture.joint_names.append("panda_finger_joint1")
    posture.joint_names.append("panda_finger_joint2")

    #Set them as open. 
    pos = JointTrajectoryPoint()
    pos.positions.append(object_width/2-0.01) #pos finger 1. box wid/2
    pos.positions.append(object_width/2-0.01) #pos finger 2
    pos.time_from_start=rospy.Duration(0.5)
    posture.points.append(pos)


def openGripperPose(posture):
    #auxiliary function needed for pick
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
def handle_graps(req):
    #this will create an element of type grasp
    object_name=req.object_name

    object_pose=Pose()
    object_pose=Query_object_Pose(object_name)

    object_dimensions=[0,0,0]
    #get object dimensions
    if object_name=="block":
        object_dimensions=[0.045, 0.045, 0.2]
    
    print object_dimensions

    #graps 1
    #create object grasp
    g = Grasp()
    g.id=object_name

    #GRASP POSE: position of object in end efector seen from world
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = "panda_link0"
    grasp_pose.pose.position.x = object_pose.position.x -(object_dimensions[0]/2) -0.058-0.02 #(box position- (half box lenght- distance distance b/w panda_link8 and palm of eef (0.058) - some extra padding)))
    grasp_pose.pose.position.y = object_pose.position.y
    grasp_pose.pose.position.z = object_pose.position.z
    #orientation in quaternion
    q = quaternion_from_euler(-pi/2,-pi/4,-pi/2)
    grasp_pose.pose.orientation.x = q[0]
    grasp_pose.pose.orientation.y = q[1]
    grasp_pose.pose.orientation.z = q[2]
    grasp_pose.pose.orientation.w = q[3]
    g.grasp_pose = grasp_pose #set grasp pose

    #pre-grasp approach
    g.pre_grasp_approach.direction.header.frame_id = "panda_link0"
    g.pre_grasp_approach.direction.vector.x = 1.0
    g.pre_grasp_approach.min_distance = 0.095
    g.pre_grasp_approach.desired_distance = 0.115
    openGripperPose(g.pre_grasp_posture) #posture before grasp

    #posture during grasp
    closeGripperPose(g.grasp_posture,object_dimensions[0])

    #post-grasp retreat
    g.post_grasp_retreat.direction.header.frame_id = "panda_link0"
    g.post_grasp_retreat.direction.vector.z = 1.0
    g.post_grasp_retreat.min_distance = 0.1
    g.post_grasp_retreat.desired_distance= 0.25


    #print "Returning grasp [%s , %s , %s]"%(g.grasp_pose.pose.position.x, g.grasp_pose.pose.position.y, g.grasp_pose.pose.position.z)
    return g

#------------------------------------
#--------SERVER CREATION---------
#------------------------------------
def compute_grasps_server():
    #init server node
    rospy.init_node('compute_grasps_server')
    s = rospy.Service('ComputeGrasps', computegraps, handle_graps) #service name, type, callback function
    #print "Ready to compute the grasp for an object"
    #keep running the service
    rospy.spin()


#------------------------------------
#--------MAIN---------
#------------------------------------
if __name__ == "__main__":
    compute_grasps_server()