#!/usr/bin/env python2

#the idea of this code is to update rviz using the position of an object in gazebo
#there are 2 ways, call a service and get the position and the modify rviz,
#or a subscriber that will be continuosly updating this.

import sys

import rospy
from gazebo_msgs.msg import ModelStates
from moveit_commander import PlanningSceneInterface, roscpp_initialize
from geometry_msgs.msg import PoseStamped, Pose

##getting info from a service
# if __name__ == "__main__":
#     rospy.init_node('get_robot_position')
#     rospy.wait_for_service('/gazebo/get_model_state')
#     try:
#         gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
#         resp1 = gms(model_name="box")
#     except rospy.ServiceException, e:
#         rospy.logerr('Error on calling service: %s',str(e))
#         print "Service call failed: %s"%e    
#     print resp1.pose

##compare poses
def compare_poses(pose1,pose2):
    #return a boolean that say if 2 poses are the same
    #we use round because e-17 numbers cause false negatives
    x_pos=round(pose1.position.x,7)==round(pose2.position.x,7)
    y_pos=round(pose1.position.y,7)==round(pose2.position.y,7)
    z_pos=round(pose1.position.z,7)==round(pose2.position.z,7)
    x_or=round(pose1.orientation.x,7)==round(pose2.orientation.x,7)
    y_or=round(pose1.orientation.y,7)==round(pose2.orientation.y,7)
    z_or=round(pose1.orientation.z,7)==round(pose2.orientation.z,7)
    w_or=round(pose1.orientation.w,7)==round(pose2.orientation.w,7)
    return x_pos and y_pos and z_pos and x_or and y_or and z_or and w_or
    

##info as a subscriber
def callback(data):
    object_name="block" #whis will be a parameter of the node
    object_size=(0.045, 0.045, 0.2)

    #extract the pose of the object in gazebo (in message received)
    #look the object in the message
    gazebo_pose=Pose() #initialize variable in case loop fail
    for j in range (0,len(data.name)):
        if data.name[j]==object_name:
            gazebo_pose=data.pose[j]
            break
    
    #define the new pose of the object
    object_new_pose =PoseStamped()
    object_new_pose.header.frame_id = "panda_link0"
    object_new_pose.pose=gazebo_pose

    #scene.remove_world_object(object_name)
    #create the object
    #scene.add_box(object_name, object_new_pose, size=(0.045, 0.045, 0.2))
    #verify if the object is present in rviz
    rviz_objects=scene.get_known_object_names()
    #if there aren't objects, create it
    if len(rviz_objects)==0:
        scene.add_box(object_name, object_new_pose, size=(0.045, 0.045, 0.2))
    else:
        #check if the object is present.
        for i in range (0,len(rviz_objects)):
            #if completed list, but object wasn't present, create it

            if rviz_objects[i]==object_name:
                #check if the rviz object and gazebo object is in the same pose
                #first lets get rviz pose
                pos_list_rviz=scene.get_object_poses([object_name])
                pos_rviz=Pose()
                pos_rviz.position= pos_list_rviz[object_name].position
                pos_rviz.orientation= pos_list_rviz[object_name].orientation
                #compare the poses
                if compare_poses(pos_rviz,gazebo_pose):
                   # print "same pose"
                    break
                else:
                    #print "different pose"
                    #if they are different, update rviz pose
                    #remove object from world
                    scene.remove_world_object(object_name)
                    #create the object
                    scene.add_box(object_name, object_new_pose, size=(0.045, 0.045, 0.2))
                    #break if we found the object we where looking for
                    break
            if i==len(rviz_objects)-1 and not(i==0):
                #print "add new object"
                #if object is not in rviz list, create it for first time
                scene.add_box(object_name, object_new_pose, size=(0.045, 0.045, 0.2))

                




    
def listener():
    # create the node of the subcriber
    rospy.init_node('box_subscriber_node', anonymous=True)

    #create the initial object in rviz

    #create subscriber (topic, message type, callback function)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    #this will be parameters of the node. can be obtainez from gazebo or xacro?
    object_name="block"
    object_size=(0.045, 0.045, 0.2)

    ## initialize `moveit_commander`
    roscpp_initialize(sys.argv)
    
    #init planning scene interface
    scene =PlanningSceneInterface()

    #initialize listener
    listener()

