#!/usr/bin/env python3

# This Python script implements a ROS service server that adds two integers.
# It uses the AddTwoInts service provided by the rospy_tutorials package.

import rospy
from rospy_tutorials.srv import AddTwoInts



def handle_add_two_ints(req):

    result = req.a + req.b

    # Log an informational message about the operation
    rospy.loginfo("Sum of " + str(req.a) + " and " + str(req.b) + " is = " + str(result))
    return result



if __name__ == '__main__':
    # Initialize a ROS node and name it
    rospy.init_node('add_two_ints_server')

    # Log an informational message about the node creation
    rospy.loginfo("Add two ints server node created")

    # Create the service server and associate it with the handle_add_two_ints function
    service = rospy.Service("/add_two_ints", AddTwoInts, handle_add_two_ints)

    # Log an informational message about the service server startup
    rospy.loginfo("Service Server has been started")

    # Keep the node running to handle service requests
    rospy.spin()
