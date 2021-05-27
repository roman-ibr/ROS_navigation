#! /usr/bin/env python

# ===============================================================
# Name and Surname: Roman Ibrahimov
# Class: ME 597 Autonomous Systems 
# Email Address: ibrahir@purdue.edu
#
# Program Description: 
# The program creates a "/save_spot" service. When the requested labels are equal 
# to the given ones, the robot saves its current position in spots.txt file. 
#
# References: 
# 1. https://www.theconstructsim.com/ros-qa-121-how-to-write-robot-poses-to-a-file/
# 2. https://github.com/srnand/Navigating-the-Summit-Robot 
# 3. https://bitbucket.org/theconstructcore/teb_navigation_course_solutions
# 
# =================================================================

import rospy
# newly-created service message 
from my_summit_localization.srv import MyServiceMessage, MyServiceMessageResponse
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
import time


class SaveSpots(object):
    # /save_spot service initialization
    def __init__(self, srv_name='/save_spot'):
        self._srv_name = srv_name
        self._pose = PoseWithCovarianceStamped()
        self.detection_dict = {"turtle":self._pose, "table":self._pose, "room":self._pose}
        #creating the above-mentioned service
        self._my_service = rospy.Service(self._srv_name, MyServiceMessage , self.srv_callback)
        #subscriber to get robot's position
        self._pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped , self.sub_callback)

    def sub_callback(self, msg):
        self._pose = msg
    
    def srv_callback(self, request):
        #service request
        label = request.label
        #service response, which are navigation_successfull and message
        #their types are boolean and string 
        response = MyServiceMessageResponse()

        # if the service request is "turtle", the robot's current position is saved and service message is given
        if label == "turtle":
            self.detection_dict["turtle"] = self._pose
            response.message = "Saved Pose for turtle spot"
        # if the service request is "table", the robot's current position is saved and service message is given
        elif label == "table":
            self.detection_dict["table"] = self._pose
            response.message = "Saved Pose for table spot"
        # if the service request is "room", the robot's current position is saved and service message is given   
        elif label == "room":
            self.detection_dict["room"] = self._pose
            response.message = "Saved Pose for room spot"
        # if the user request "end", the recorded positions will be saved in "spots.txt"
        elif label == "end":
            with open('spots.txt', 'w') as file:
                
                for key, value in self.detection_dict.iteritems():
                    if value:
                        file.write(str(key) + ':\n----------\n' + str(value) + '\n===========\n')
                #response message if it is wrriten successfully 
                response.message = "Successfully written to spots.txt file"
        #otherwise respond with the following message        
        else:
            response.message = "Request room, table, or turtle."
        
        #if everything goes well, set response bool mes to true
        response.navigation_successfull = True
        
        return response


if __name__ == "__main__":
    #node initialization
    rospy.init_node('spot_recorder', log_level=rospy.INFO) 
    #creating an object 
    save_spots_object = SaveSpots()
    #keeping the service open
    rospy.spin() 















# import rospy
# from my_summit_localization.srv import MyServiceMessage, MyServiceMessageResponse
# from geometry_msgs.msg import Pose, PoseWithCovarianceStamped


# class SaveSpot():
#     def __init__(self):
#         self._pose = PoseWithCovarianceStamped()
#         self.detection_dict = {"turtle":self._pose, "table":self._pose, "room":self._pose}
#         self.my_service = rospy.Service('/save_spot',MyServiceMessage,self.callBack)
#         self._pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped , self.sub_callBack)
    
#     def sub_callBack(self, request):
#         self._pose=request
    
#     def callBack(self, request):
#         if request.label=="turtle":
#             self.detection_dict["turtle"]=self._pose
#             MyServiceMessageResponse.message = "Saved Pose for turtle spot"
#         elif request.label=="table":
#             self.detection_dict["table"]=self._pose
#             MyServiceMessageResponse.message = "Saved Pose for table spot"
#         elif request.label=="room":
#             self.detection_dict["room"]=self._pose
#             MyServiceMessageResponse.message = "Saved Pose for room spot"
#         elif request.label=="end":
#             f=open('spots.txt','w')
#             for key, value in self.detection_dict.iteritems():
#                     if value:
#                         f.write(str(key) + ':\n----------\n' + str(value) + '\n===========\n')
#             f.close()
#             response.message = "Written Poses to spots.txt file"
#         MyServiceMessageResponse.navigation_successful=True
#         return MyServiceMessageResponse

# rospy.init_node('spot_recorder')

# save_spot = SaveSpot()
# rospy.spin()
