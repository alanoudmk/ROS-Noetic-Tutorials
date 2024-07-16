
	#!/usr/bin/env python3

	import rospy
	from std_msgs.msg import String


	def callback_receive_radio_data(msg):     # Define a callback function to handle received messages
		rospy.loginfo("Message received: ")     # Log an informational message
		rospy.loginfo(msg)                      # Log the received message




	if __name__ == '__main__':

     # Initialize a ROS node with the name 'smartphone'
		rospy.init_node('smartphone')          

    # Create a subscriber that listens to the '/robot_news_radio' topic and calls the callback function
		sub = rospy.Subscriber("/robot_news_radio", String, callback_receive_radio_data)

     # Enter the ROS event loop and keep the node running until it is shut down
		rospy.spin()
