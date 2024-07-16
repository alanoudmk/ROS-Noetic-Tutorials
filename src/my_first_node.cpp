#include <ros/ros.h>


int main( int argc, char **argv){
  
  	ros::init(argc, argv, "my_first_cpp_node");    // Initialize the ROS node with the name 'my_first_cpp_node'
  
  	ros::NodeHandle nh;                             // Create a node handle to interact with ROS
  
  	ROS_INFO("Node has been started");              // Log an informational message
  
  	ros::Rate rate(10);
  
	  while( ros::ok()){       // Run the loop until the node is shut down
		  ROS_INFO("Hello");     // Log an informational message
		  rate.sleep();          // Sleep for the remainder of the 0.1 second loop iteration
	  }
  
  }
