#!/usr/bin/env python

# import the required dependencies
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from navigation_exam.srv import SaveSpot, SaveSpotResponse
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import yaml

class SpotRecorder():
    def __init__(self):
        # Variable Initialization
        self.pose = Pose()
        self.pose_dict_list = []
        self.filename = ""
        self.count = 0
        self.finish_server = False
        # Start where the server will live
        rospy.init_node('spot_recorder')
        # Wait for the /odom_combined topic to become available
        rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped)
        # Subscribe to the /odom_combined topic
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.__amcl_pose_sub)
        # Define the service and the handler to respond to incoming requests
        self.s = rospy.Service('save_spot', SaveSpot, self.__handle_save_spot) 
        self.rate = rospy.Rate(5)

    
    def __handle_save_spot(self,req):
        str_msg = ""
        if req.label == 'end':
            str_msg = "File %s saved with %d new labels"%(self.filename,self.count) 
            print(str_msg)
            self.finish_server = True
        else:
            self.pose_dict_list[0]['label'] = req.label
            fh = open(req.filename,'a+')
            yaml.dump(self.pose_dict_list, fh)
            self.count += 1
            str_msg = "Saving %s label into file %s"%(req.label,req.filename)
            print(str_msg)
        return SaveSpotResponse(True, str_msg)

    def __pose_to_dict(self,pose):
        px = pose.position.x
        py = pose.position.y
        pz = pose.position.z
        ox = pose.orientation.x        
        oy = pose.orientation.y
        oz = pose.orientation.z
        ow = pose.orientation.w
        pose_dict = {'label':'',
                     'position':{
                     'x':px,
                     'y':py,
                     'z':pz},
                     'orientation':{
                     'x':ox,
                     'y':oy,
                     'z':oz,
                     'w':ow}}
        return [pose_dict]

    def __amcl_pose_sub(self,msg):
        self.pose = msg.pose.pose
        self.pose_dict_list = self.__pose_to_dict(self.pose)       

    def server(self):
        # Keep the node runnign waiting for a request
        while not rospy.is_shutdown():
            if(self.finish_server == True):
                break
            self.rate.sleep()
            

if __name__=="__main__":
    try:
        SpotRecorder().server()
    except:
        print("Unexpected Error while running SaveSpot server")

