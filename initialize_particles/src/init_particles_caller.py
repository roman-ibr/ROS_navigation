#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyRequest # Import the service message used by the service /gazebo/delete_model
import sys 

rospy.init_node('service_client') # Initialise a ROS node with the name service_client
rospy.wait_for_service('/global_localization') # Wait for the service client /gazebo/delete_model to be running
disperse_particles_service = rospy.ServiceProxy('/global_localization', Empty) # Create the connection to the service
msg = EmptyRequest() # Create an object of type DeleteModelRequest
result = disperse_particles_service(msg) # Send through the connection the name of the object to be deleted by the service
print (result) # Print the result given by the service called