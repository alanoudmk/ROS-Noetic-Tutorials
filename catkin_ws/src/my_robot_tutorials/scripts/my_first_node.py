#!/usr/bin/env python3

import rospy


if __name__ == '__main__':

  # Initialize a ROS node with the name 'my_first_python_node'
  rospy.init_node('my_first_python_node')

   # Log an informational message
   rospy.loginfo("This node has been started")


  rate = rospy.Rate(10)

 
  while not rospy.is_shutdown():       # Run the loop until the node is shutdown
    	rospy.loginfo("Hello")           # Log an informational message
    	rate.sleep()                     # Sleep for the remainder of the 0.1 second loop iteration
