#!/usr/bin/env python2

#demo based on https://github.com/RethinkRobotics/sawyer_simulator/blob/master/sawyer_sim_examples/scripts/ik_pick_and_place_demo.py
# https://github.com/frankaemika/external_gripper_example/blob/master/panda_with_robotiq_gripper_example/scripts/panda_with_robotiq_gripper_example.py
# https://github.com/ros-planning/moveit_tutorials/blob/melodic-devel/doc/pick_place/src/pick_place_tutorial.cpp
# 
#Written by Gabriela Bravo -Illanes

import sys
import copy
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import  Grasp, GripperTranslation, PlaceLocation
from geometry_msgs.msg import PoseStamped, Pose
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

from gazebo_msgs.srv import GetModelState #gazebo service

from panda_demo.srv import * #messages needed to use the services in the package

import ros
import RobotCommander

node_prefix = 'panda_pick_place_demo: '  # To label log messages coming from this file.

##function to set objects in the scene
def addSceneObjects(target_scene):
    #create a Table1
    # table1_pose = PoseStamped()
    # table1_pose.header.frame_id = "panda_link0"
    # table1_pose.pose.orientation.w = 1.0
    # table1_pose.pose.position.x = 0.5
    # table1_pose.pose.position.y = 0 
    # table1_pose.pose.position.z = 0.2 
    # table1_name = "table1"
    # target_scene.add_box(table1_name, table1_pose, size=(0.2, 0.4, 0.4))

    #create a Table2
    # table2_pose = PoseStamped()
    # table2_pose.header.frame_id = "panda_link0"
    # table2_pose.pose.orientation.w = 1.0
    # table2_pose.pose.position.x = 0
    # table2_pose.pose.position.y = 0.5 
    # table2_pose.pose.position.z = 0.2 
    # table2_name = "table2"
    # target_scene.add_box(table2_name, table2_pose, size=(0.4, 0.2, 0.4))

    #create a box
    box_pose = PoseStamped()
    box_pose.header.frame_id = "panda_link0"
    box_name = "block"
    box_pose.pose=Query_object_Pose( box_name)
    target_scene.add_box(box_name, box_pose, size=(0.045, 0.045, 0.2))

    #create a Table1
    table1_pose = PoseStamped()
    table1_pose.header.frame_id = "panda_link0"
    table1_pose.pose.orientation.w = 1.0
    table1_pose.pose.position.x = 0.75
    table1_pose.pose.position.y = 0 
    table1_pose.pose.position.z = 0 
    table1_name = "table1"
    target_scene.add_box(table1_name, table1_pose, size=(0.913, 0.913, 0.04))

    rospy.sleep(1)



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
    except rospy.ServiceException as e:
        rospy.logerr('Error on calling service: %s',str(e))
        print("Service call failed: %s"%e     )
    print(resp1.pose)

    #manual pose
    box_pose=Pose()
    box_pose.position.x=0.4225 
    box_pose.position.y=0.1265
    box_pose.position.z=0.1213
    #return box_pose #manual pose
    return resp1.pose #gazebo pose


def main():
    ##---INITIALIZATI pi/2ONS--##
   
    #init this node
    rospy.init_node('pick_place_demo', anonymous=True)

    #init planning scene interface
    scene = PlanningSceneInterface()
    rospy.sleep(1) #time to process


    ##---INITIAL SET UP--##
    # clean the scene
    # scene.remove_world_object("table1")
    # scene.remove_world_object("table2")
   # scene.remove_world_object("block")
    
    #start robot
    Panda_R = RobotCommander.Robot()

   # scene.remove_attached_object(Panda_R.eef_link, name="block")

    #place objects on initial configuration
    addSceneObjects(scene)



    rospy.sleep(1)

    


    #code
    try:
        print ""
        print "----------------------------------------------------------"
        print "Welcome to the Pick and place demo"
        print "----------------------------------------------------------"
        print "Press Ctrl-D to exit at any time"
        print ""
        print "============ Press `Enter` to begin the demo ..."

        # pos=Query_object_Pose(scene, "box")
        # print pos

        # width=Query_object_width(scene, "box")
        # print width

        #ser_ops=scene.get_object_poses(["box"])
        #print ser_ops["box"].position.x

        #ser_ops=scene.get_objects(["box"])
        #print ser_ops["box"].primitives[0].dimensions

        #ser_ops=Panda_R.get_eef_pose()
        #print ser_ops

        print "============ Press `Enter` to go home position..."
        raw_input()
        #Go home position
        Panda_R.go_home()

        print "============ Press `Enter` to pick ..."
        raw_input()
        #box_pose=Query_object_Pose(scene,"block")
        #box_dimensions=Query_object_dimensions(scene,"block")
        Panda_R.pick("block")
        rospy.sleep(1)
        
        print "============ Press `Enter` to place ..."
        raw_input()

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
        Panda_R.place(place_pose)
        rospy.sleep(1)

        # print "============ Press `Enter` to go to a pose ..."
        # raw_input()
        # pose_goal = Pose()
        # pose_goal.orientation.w = 1.0
        # pose_goal.position.x = 0
        # pose_goal.position.y = 0
        # pose_goal.position.z = 2
        # Panda_R.go_to_pose_goal(pose_goal)


        #print "============ Press `Enter` to open gripper ..."
        #raw_input()
        #Panda_R.openGripper()
        

        print "============ Demo complete!"
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return




    


if __name__ == '__main__':
    sys.exit(main())
