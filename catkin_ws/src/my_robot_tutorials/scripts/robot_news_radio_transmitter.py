#!/usr/bin/env python3

import rospy 
from std_msgs.msg import String       # Import the String message type from the standard ROS message types


if __name__ == '__main__':
		rospy.init_node('robot_news_radio_transmitter')                    # Initialize a ROS node with the name 'robot_news_radio_transmitter'
		pub = rospy.Publisher("/robot_news_radio", String, queue_size=10)  # Create a publisher that publishes to the '/robot_news_radio' topic with a queue size of 10
		rate = rospy.Rate(2)                                               # Create a rate object to control the loop frequency (2 Hz)
		
   while not rospy.is_shutdown():
			msg = String()
			msg.data = "Hi, this is Tom from the Robot News Radio !"
			pub.publish( msg )    # Publish the message to the '/robot_news_radio' topic
			rate.sleep()
     
		rospy.loginfo("Node was Stopped")                                   # Log an informational message when the node is stoppe
