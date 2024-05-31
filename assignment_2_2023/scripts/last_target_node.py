#!/usr/bin/env python
"""
.. module:: last_target_node
   :platform: Unix
   :synopsis: 
   
   A service node that, when called, returns the coordinates of the last target sent by the user.
   
  
.. moduleauthor:: Cheima Ferdjallah cheimaferdjallah@gmail.com

Subscribes to:
    /reaching_goal/goal: Subscribes to the /reaching_goal topic in order to receive updates on the last target position.

Service:
    /get_last_target_coordinates: Service server that gives the last target coordinates set by the user.
    The service is defined in the assignment_2_2023 package.
"""

import rospy
from assignment_2_2023.srv import Coordinates, CoordinatesResponse
from assignment_2_2023.msg import PlanningActionGoal

# Global variable to store the last target coordinates
last_target_coordinates = {'x': 0.0, 'y': 0.0}
"""
Dictionary to store the last target coordinates.
"""
def goal_callback(msg):
    """
    Callback function to get the target coordinates from the PlanningActionGoal message.
    """
    # Extract target coordinates from the PlanningActionGoal message
    last_target_coordinates['x'] = msg.goal.target_pose.pose.position.x
    last_target_coordinates['y'] = msg.goal.target_pose.pose.position.y


def coordinates_service_callback(req):
    """
    Service callback function to handle requests for the last target coordinates.
    """
    # Service callback function to handle requests
    response = CoordinatesResponse()
    response.x = last_target_coordinates['x']
    response.y = last_target_coordinates['y']
    return response

if __name__ == "__main__":
    """
    Main function to initialize the ROS node, subscribe to the /reaching_goal/goal topic, and create a service server.
    """
    rospy.init_node('last_target_node')

    # Subscribe to the /reaching_goal/goal topic
    rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, goal_callback)

    # Create a service server
    service = rospy.Service('/get_last_target_coordinates', Coordinates, coordinates_service_callback)

    rospy.spin()
