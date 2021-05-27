#!/usr/bin/env python

# import the required dependencies
# Import general python libraries
import numpy as np
# Import general ros library for python
import rospy
# Brings in the SimpleActionClient
import actionlib
# Import the necessary message to interact with the move_base action server.
import move_base_msgs.msg
# Import the required library to use message classes 
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped

import tf
import yaml

class SendGoalsNode():
    def __init__(self):
        # Variable Initialization
        self.pose = Pose()
        self.pose_dict_list = []
        # Start where the server will live
        rospy.init_node('send_goals_node')
        # Load all the spots stored in the yaml file
        self.pose_array = rospy.get_param("/spots_recorded")
        # Wait for the /odom_combined topic to become available
        rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped)
        # Subscribe to the /odom_combined topic
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.__amcl_pose_sub)
        # Define the execution rate of the node
        self.rate = rospy.Rate(5)

    def __create_pose_stamped_msg(self,pose):
        # Create a pose stamped object and populate the fields
        goalMsg = PoseStamped()
        goalMsg.header.frame_id = 'map'
        # Position
        goalMsg.pose.position.x = pose.position.x
        goalMsg.pose.position.y = pose.position.y
        goalMsg.pose.position.z = pose.position.z
        # Orientation
        goalMsg.pose.orientation.x = pose.orientation.x
        goalMsg.pose.orientation.y = pose.orientation.y
        goalMsg.pose.orientation.z = pose.orientation.z
        goalMsg.pose.orientation.w = pose.orientation.w
        # ROS time stamp
        goalMsg.header.stamp = rospy.Time.now() 
        
        return goalMsg
    
    def __move_base_client(self,pose):
        # Creates the SimpleActionClient, passing the type of the action
        # (MoveBaseAction) to the constructor.
        client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)  
        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()        
        # Creates a goal to send to the action server.
        goal = move_base_msgs.msg.MoveBaseGoal(target_pose=self.__create_pose_stamped_msg(pose))
        # Sends the goal to the action server.
        #print(goal)
        client.send_goal(goal) 
        # Waits for the server to finish performing the action.
        client.wait_for_result()   
        # Prints out the result of executing the action
        return client.get_result()

    
    def __handle_send_goal_label(self,label):
        # Look for label in spots file
        pose = self.__get_pose_from_label(label)
        # Move the base throught the action service move_base
        resp = self.__move_base_client(pose)
        # Decide the response based on the Action Server response
        return resp 

    def __get_pose_from_label(self,label):
        pose = self.__yaml_to_pose(self.pose_array[label])
        return pose

    def __yaml_to_pose(self,pose_dict):
        pose = Pose()
        # Position
        pose.position.x = pose_dict['position']['x']
        pose.position.y = pose_dict['position']['y']
        pose.position.z = pose_dict['position']['z']
        # Orientation 
        pose.orientation.x = pose_dict['orientation']['x']
        pose.orientation.y = pose_dict['orientation']['y']
        pose.orientation.z = pose_dict['orientation']['z']
        pose.orientation.w = pose_dict['orientation']['w']

        return pose

    def __amcl_pose_sub(self,msg):
        self.pose = msg.pose.pose
        

    def run(self):
        # Keep the node runnign waiting for a request
        while not rospy.is_shutdown():
            # Keep looping
            for pose_d in self.pose_array:
                self.__handle_send_goal_label(pose_d)
            self.rate.sleep()            

if __name__=="__main__":
    #try:
    SendGoalsNode().run()
    #except:
    #    print("Unexpected Error while running SendGoalPoseServer server")