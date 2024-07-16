#include <ros/ros.h>
#include <rospy_tutorials/AddTwoInts.h>



int main(int argc, char **argv) {
  
  ros::init(argc, argv, "add_two_ints_client");   // Initialize the ROS node with the name 'add_two_ints_client'
  ros::NodeHandle nh;                             // Create a node handle to interact with ROS

  // Create a service client to call the 'add_two_ints' service
  ros::ServiceClient client = nh.serviceClient<rospy_tutorials::AddTwoInts>("/add_two_ints");

  // Prepare the request for the 'add_two_ints' service
  rospy_tutorials::AddTwoInts srv;
  srv.request.a = 12;
  srv.request.b = 5;



  
  // Call the 'add_two_ints' service and process the response
  if (client.call(srv)) {
    // The service call was successful, so process the response
    ROS_INFO("Returned Sum is : %d", (int)srv.response.sum);

  } else {
    // The service call failed, so print a warning
    ROS_WARN("Service Call FAILED");
  }

  
}
