#!/usr/bin/env python2

#File with functions to control the robot
#using simple comands
import sys
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import  Grasp, GripperTranslation, PlaceLocation
from geometry_msgs.msg import PoseStamped, Pose
from panda_demo.srv import robotRequestPose,robotRequestPoseResponse, robotSimpleCommand, robotSimpleCommandResponse, robotPickPlace, robotPickPlaceResponse, computegraps
from math import pi
from trajectory_msgs.msg import JointTrajectoryPoint


#------------------------------------
#--------ROBOT CLASS---------
#------------------------------------
##Class with all the importatn parameters of the robot 
class Robot(object):
    def __init__(self):
        ## First initialize `moveit_commander`
        roscpp_initialize(sys.argv)
        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints). 
        move_group = MoveGroupCommander("panda_arm_hand")
        move_group_arm = MoveGroupCommander("panda_arm")

        move_group_arm.set_planning_time(120)

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

    def get_eef_pose(self):
        #function that get current end effector pose
        return self.move_group_arm.get_current_pose().pose

    def go_to_pose_goal(self,pose_goal):
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:

        self.move_group_arm.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan =self.move_group_arm.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group_arm.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group_arm.clear_pose_targets()

    def go_home(self):
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
        ## thing we want to do is move it to a slightly better configuration.
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -0.785
        joint_goal[2] = 0
        joint_goal[3] = -2.356
        joint_goal[4] = 0
        joint_goal[5] = 1.571
        joint_goal[6] = 0.785
        joint_goal[7] = 0.04 #finger1
        joint_goal[8] = 0.04 #finger 2


        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()
    
    def openGripper(self):
        #set finger in the maximum position
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[7] = 0.04 #finger1
        joint_goal[8] = 0.04 #finger 2

        #plan and go
        self.move_group.go(joint_goal, wait=True)

        #stop ensure no residual movement
        self.move_group.stop()

        

    def closeGripper(self):
        #set finger in the minimum position
        joint_goal = self.move_group.get_current_joint_values()
       
        joint_goal[7] = 0 #finger1
        joint_goal[8] = 0 #finger 2
        
        #plan and go
        self.move_group.go(joint_goal, wait=True)

        #stop ensure no residual movementroscpp
        self.move_group.stop()

    
    def pick(self,object_name):
          
        #surface in contact with object
        #g.allowed_touch_objects = ["table1","box","table2"]
        self.move_group_arm.set_support_surface_name("table1")

        #get list with possible object grasp
        grasps=compute_grasps_client(object_name)        
        
        # pick the object
        self.move_group_arm.pick(object_name, grasps)
        


    def place(self,place_pose):
        #create a place location msg
        pl=PlaceLocation()

        #Setting place location pose
        # +++++++++++++++++++++++++++
        place_pose_stamped = PoseStamped()
        place_pose_stamped.header.frame_id = "panda_link0"
        place_pose_stamped.pose=place_pose
        pl.place_pose=place_pose_stamped #set place location

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
        openGripperPose(pl.post_place_posture)


        # Set support surface as table2.
        self.move_group_arm.set_support_surface_name("table2")
        # Call place to place the object using the place locations given.
        self.move_group_arm.place("block", pl)

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

