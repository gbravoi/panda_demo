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

import ros

node_prefix = 'panda_pick_place_demo: '  # To label log messages coming from this file.

##function to set objects in the scene
def addSceneObjects(target_scene):
    #create a Table1
    table1_pose = PoseStamped()
    table1_pose.header.frame_id = "panda_link0"
    table1_pose.pose.orientation.w = 1.0
    table1_pose.pose.position.x = 0.5
    table1_pose.pose.position.y = 0 
    table1_pose.pose.position.z = 0.2 
    table1_name = "table1"
    target_scene.add_box(table1_name, table1_pose, size=(0.2, 0.4, 0.4))

    #create a Table2
    table2_pose = PoseStamped()
    table2_pose.header.frame_id = "panda_link0"
    table2_pose.pose.orientation.w = 1.0
    table2_pose.pose.position.x = 0
    table2_pose.pose.position.y = 0.5 
    table2_pose.pose.position.z = 0.2 
    table2_name = "table2"
    target_scene.add_box(table2_name, table2_pose, size=(0.4, 0.2, 0.4))

    #create a box
    box_pose = PoseStamped()
    box_pose.header.frame_id = "panda_link0"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.5
    box_pose.pose.position.y = 0 
    box_pose.pose.position.z = 0.5 
    box_name = "box"
    target_scene.add_box(box_name, box_pose, size=(0.02, 0.02, 0.2))

    rospy.sleep(1)


##Class with all the thing the robot can do
class Panda_Robot_demo(object):
    def __init__(self):
        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints). 
        move_group = MoveGroupCommander("panda_arm_hand")
        move_group_arm = MoveGroupCommander("panda_arm")

        move_group_arm.set_planning_time(45)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = RobotCommander()
        #end effector link
        eef_link = move_group_arm.get_end_effector_link()

        # Misc variables
        self.robot = robot
        self.move_group= move_group
        self.move_group_arm= move_group_arm
        self.eef_link=eef_link



    def go_home(self):
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
        ## thing we want to do is move it to a slightly better configuration.
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/3
        joint_goal[6] = 0
        joint_goal[7] = 0.04 #finger1
        joint_goal[8] = 0.04 #finger 2

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()


    def go_to_pose_goal(self):
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        self.move_group_arm.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan =self.move_group_arm.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group_arm.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group_arm.clear_pose_targets()
    

    def openGripper(self):
        #set finger in the maximum position
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[7] = 0.04 #finger1
        joint_goal[8] = 0.04 #finger 2

        #plan and go
        self.move_group.go(joint_goal, wait=True)

        #stop ensure no residual movement
        self.move_group.stop()


    def openGripperPose(self, posture):
        posture.joint_names=[]
        posture.joint_names.append("panda_finger_joint1")
        posture.joint_names.append("panda_finger_joint2")

        #Set them as open. 
        pos = JointTrajectoryPoint()
        pos.positions.append(0.04) #pos finger 1
        pos.positions.append(0.04) #pos finger 2
        pos.time_from_start=rospy.Duration(0.5)
        posture.points.append(pos)
        
        

    def closeGripper(self):
        #set finger in the minimum position
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[7] = 0 #finger1
        joint_goal[8] = 0 #finger 2

        #plan and go
        self.move_group.go(joint_goal, wait=True)

        #stop ensure no residual movementroscpp
        self.move_group.stop()


    def closeGripperPose(self,posture,object_width):
        posture.joint_names=[]
        posture.joint_names.append("panda_finger_joint1")
        posture.joint_names.append("panda_finger_joint2")

        #Set them as open. 
        pos = JointTrajectoryPoint()
        pos.positions.append(object_width/2) #pos finger 1. box wid/2
        pos.positions.append(object_width/2) #pos finger 2
        pos.time_from_start=rospy.Duration(0.5)
        posture.points.append(pos)
        
        
    
    def pick(self):
        ##-----------GRAPS---------------##
        #set all grasp poses
        #first create an array where all graps are going to be saved
        grasps = []

        #graps 1, box
        #create object grasp
        g = Grasp()
        g.id="box_grasp"

        #GRASP POSE: position of object in end efector seen from world
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = "panda_link0"
        grasp_pose.pose.position.x = 0.4 #(box position- (half box lenght- distance distance b/w panda_link8 and palm of eef (0.058) - some extra padding)))
        grasp_pose.pose.position.y = 0
        grasp_pose.pose.position.z = 0.5
        #orientation in quaternion
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
        self.openGripperPose(g.pre_grasp_posture) #posture before grasp

        #posture during grasp
        self.closeGripperPose(g.grasp_posture,0.02)

        #post-grasp retreat
        g.post_grasp_retreat.direction.header.frame_id = "panda_link0"
        g.post_grasp_retreat.direction.vector.z = 1.0
        g.post_grasp_retreat.min_distance = 0.1
        g.post_grasp_retreat.desired_distance= 0.25
    
        #surface in contact with object
        #g.allowed_touch_objects = ["table1","box","table2"]
        self.move_group_arm.set_support_surface_name("table1")
        
    
        # append the grasp to the list of grasps
        grasps.append(g)
        rospy.sleep(2)
        
        
        # pick the object
        self.move_group_arm.pick("box", grasps)


    def place(self):
        #create a place location msg
        pl=PlaceLocation()

        #Setting place location pose
        # +++++++++++++++++++++++++++
        place_pose = PoseStamped()
        place_pose.header.frame_id = "panda_link0"
        place_pose.pose.position.x = 0 
        place_pose.pose.position.y = 0.5
        place_pose.pose.position.z = 0.5
        #orientation in quaternion
        q = quaternion_from_euler(0,0,pi/2)
        place_pose.pose.orientation.x = q[0]
        place_pose.pose.orientation.y = q[1]
        place_pose.pose.orientation.z = q[2]
        place_pose.pose.orientation.w = q[3]
        pl.place_pose=place_pose #set place location

        # Setting pre-place approach
        # ++++++++++++++++++++++++++
        #* Defined with respect to frame_id */
        pl.pre_place_approach.direction.header.frame_id = "panda_link0"
        # Direction is set as negative z axis */
        pl.pre_place_approach.direction.vector.z = -1.0
        pl.pre_place_approach.min_distance = 0.095
        pl.pre_place_approach.desired_distance = 0.115

        # Setting post-grasp retreat
        # ++++++++++++++++++++++++++
        # Defined with respect to frame_id */
        pl.post_place_retreat.direction.header.frame_id = "panda_link0"
        #* Direction is set as negative y axis */
        pl.post_place_retreat.direction.vector.y = -1.0
        pl.post_place_retreat.min_distance = 0.1
        pl.post_place_retreat.desired_distance = 0.25

        # Setting posture of eef after placing object
        # +++++++++++++++++++++++++++++++++++++++++++
        #* Similar to the pick case */
        self.openGripperPose(pl.post_place_posture)


        # Set support surface as table2.
        self.move_group_arm.set_support_surface_name("table2")
        # Call place to place the object using the place locations given.
        self.move_group_arm.place("box", pl)
        











def main():
    ##---INITIALIZATI pi/2ONS--##
    ## First initialize `moveit_commander`
    roscpp_initialize(sys.argv)

    #init this node
    rospy.init_node('pick_place_demo', anonymous=True)

    #init planning scene interface
    scene = PlanningSceneInterface()
    rospy.sleep(1) #time to process


    ##---INITIAL SET UP--##
    # clean the scene
    scene.remove_world_object("table1")
    scene.remove_world_object("table2")
    scene.remove_world_object("box")
    
    #start robot
    Panda_R = Panda_Robot_demo()

    scene.remove_attached_object(Panda_R.eef_link, name="box")

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

        
        
        print "============ Press `Enter` to go home position..."
        raw_input()
        #Go home position
        Panda_R.go_home()

        print "============ Press `Enter` to pick ..."
        raw_input()
        Panda_R.pick()
        rospy.sleep(1)
        
        print "============ Press `Enter` to place ..."
        raw_input()
        Panda_R.place()
        rospy.sleep(1)

        print "============ Press `Enter` to go home position ..."
        raw_input()
        Panda_R.go_home()


        #print "============ Press `Enter` to open gripper ..."
        #raw_input()
        #Panda_R.openGripper()
        

        print "============ Demo complete_!"
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return




    


if __name__ == '__main__':
    sys.exit(main())
