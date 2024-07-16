#!/usr/bin/env python3

# This Python script implements a ROS service client that calls the 'add_two_ints' service.
# It uses the AddTwoInts service provided by the rospy_tutorials package.

import rospy
from rospy_tutorials.srv import AddTwoInts

if __name__ == '__main__':
    # Initialize a ROS node and name it
    rospy.init_node('add_two_ints_client')

    # Wait for the 'add_two_ints' service to become available
    rospy.wait_for_service("/add_two_ints")


  
    try:
        add_two_ints = rospy.ServiceProxy("/add_two_ints", AddTwoInts)   # Create a service proxy for the 'add_two_ints' service
        response = add_two_ints(2, 6)                                    # Call the service and get the response
        rospy.loginfo("Sum is: " + str(response.sum))                    # Log the sum of the two integers
      
    except rospy.ServiceException as e:
        rospy.logwarn("Service FAILED: " + str(e))                        # Log a warning if the service call fails
