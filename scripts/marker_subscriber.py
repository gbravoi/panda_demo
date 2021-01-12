#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

class MarkerTest(object):
    def __init__(self):
        self.pose_pub = rospy.Publisher("/box_pose",Marker,queue_size=1)
    def subscriber_cb(self,data):
        object_name="block" #whis will be a parameter of the node
        #extract the pose of the object in gazebo (in message received)
        #look the object in the message
        gazebo_pose=Pose() #initialize variable in case loop fail
        for j in range (0,len(data.name)):
            if data.name[j]==object_name:
                gazebo_pose=data.pose[j]
                break
        self.publisher_fn(gazebo_pose,name=object_name)
    def marker_setup(self,data,name):
        #input is Pose tyupe
        marker = Marker()
        marker.header.frame_id = "panda_link0"
        marker.id = 1
        marker.type = marker.CUBE  #pick the right shape
        marker.action = marker.ADD
        marker.scale.x = 0.045
        marker.scale.y = 0.045
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.position = data.position
        marker.pose.orientation = data.orientation
        return marker
		
    def publisher_fn(self,data,name="block"):
        marker = self.marker_setup(data,name)
        self.pose_pub.publish(marker)
		
def main():
    rospy.init_node('marker_test_node')
    cls_obj = MarkerTest()
    rospy.Subscriber("/gazebo/model_states", ModelStates, cls_obj.subscriber_cb)
    rospy.spin()
if __name__ == '__main__':
    main()