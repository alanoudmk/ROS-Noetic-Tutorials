#include <ros/ros.h>  // Include the ROS C++ library

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "robot_news_radio_transmitter");  // Initialize the ROS node with the name "robot_news_radio_transmitter"
    
    ros::NodeHandle nh;  // Create a node handle to interact with ROS

    // Advertise the "/robot_news_radio" topic and create a publisher for the std_msgs::String message type
    ros::Publisher pub = nh.advertise<std_msgs::String>("/robot_news_radio", 10);

    ros::Rate rate(3);  // Set the loop rate to 3 Hz

    while (ros::ok()) { 
        std_msgs::String msg;  // Create a new String message
        msg.data = "Hi, this is William from The Robot News Radio! ";  // Set the message content
      
        pub.publish(msg);  // Publish the message to the "/robot_news_radio" topic
      
        rate.sleep();  // Sleep for the remaining time to maintain the 3 Hz rate
    }

}
