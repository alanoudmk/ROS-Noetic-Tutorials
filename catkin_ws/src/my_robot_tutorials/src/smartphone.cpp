#include <ros/ros.h>          // Include the ROS C++ library
#include <std_msgs/String.h>  // Include the std_msgs/String message type



// This is the callback function that will be called when a message is received on the "/robot_news_radio" topic
void callback_receive_radio_data(const std_msgs::String& msg) {
    ROS_INFO("Message Received: %s", msg.data.c_str());        // Log the received message to the console
}




int main(int argc, char **argv) {

    ros::init(argc, argv, "smartphone");    // Initialize the ROS node with the name "smartphone"
    
    ros::NodeHandle nh;                      // Create a node handle to interact with ROS

    // Subscribe to the "/robot_news_radio" topic and set the callback function to "callback_receive_radio_data"
    ros::Subscriber sub = nh.subscribe("/robot_news_radio", 1000, callback_receive_radio_data);
    
    ros::spin();                             // Enter the ROS event loop and wait for messages
}
