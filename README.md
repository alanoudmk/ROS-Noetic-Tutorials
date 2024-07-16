# What is ROS 1 Noetic ?
- ROS (Robot Operating System) Noetic is the 14th major release of the ROS software framework, released in May 2020. It is a long-term support (LTS) release, with extended maintenance and support until April 2025.

- We will be working on:
  - Ubuntu 20.04.6 LTS (Focal Fossa).
  - ROS 1 Noeatic 20.04.
  - Windows 10 and above.

## 1. Install and Setup ROS Noeatic:
- [HERE](https://github.com/alanoudmk/Install-ROS-Noetic-on-Ubuntu) is how to download it.



***

## 2. Launch Your First ROS Master:

1. Open a **Terminal**: 
   > Ctrl + Alt + T

2. Launch  ROS Master:
  - The ROS Master is the central point of the ROS system, responsible for managing communication between ROS nodes.
  - By launching roscore, you are starting the ROS Master, which allows other ROS nodes to register with it and exchange data and commands.
  
 ```
    $ roscore
```

3. limitations:
  - The ROS Master can only run as a single instance on a system at a time.
  - Attempting to run multiple instances of roscore simultaneously will result in errors, as each ROS Master requires exclusive access to the ROS communication infrastructure.
  - Single Point of Failure


***



## 3. Setting Up Your First ROS Program: (Catkin Workplace)
Catkin is the official build system used by the Robot Operating System (ROS). 
 - it empowers ROS developers to optimize their workflow by simplifying the process of building, testing, and deploying ROS-based projects. This streamlined approach makes it easier for developers to manage and share their work within the ROS community.
    
1. Open a **Terminal**: 
   > Ctrl + Alt + T

2. Create the Workspace Directory: 

```
  $ cd
  $ mkir catkin_ws
```

3. Create The Source Folder: 

  ```
    $ cd catkin_ws/
    $ mkdir src
    $ catkin_make
  ```

4. You should see the following output:

  <img src="https://github.com/alanoudmk/ROS-Noetic-Tutorials/assets/127528672/326d80bf-42e4-4c63-8681-0063294c4aa9" width="290" height="60">


5. To ensure the build was successful, run:

  ```
    $ ls
  ```

  <img src="https://github.com/alanoudmk/ROS-Noetic-Tutorials/assets/127528672/60ca20fe-cce6-4b95-b0b8-61af067d8215" width="240" height="35">

6. Source Catkin Workspace: 

 ```
   $ cd devel/
   $ source setup.bash 
  ```


7. Source Catkin Workspace in every session:

 ```
  $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
  $ gedit ~/.bashrc
 ```

- You should see the following output:
      - if Not? _Copy_ and _Paste_ it then press _SAVE_
  
   <img src="https://github.com/alanoudmk/ROS-Noetic-Tutorials/assets/127528672/f73d8e13-4e1f-4e00-9fc5-ca572e72780f" width="290" height="60">


8. Finally, The Catkin workspace is now set up, and you can start developing your first ROS program.





## 4. Creating ROS Packages:
A ROS package is the basic unit of organization in the ROS ecosystem, containing all the necessary files and dependencies for a specific functionality.
  - To run code with ROS, you will first need to create ROS packages.

 
1. Navigate to your ROS workspace's src directory:     
```
  $ cd catkin_ws/src/
  $ ls
  # CMakeLists.txt
```

2. Use the catkin_create_pkg command to create a new ROS package
- In this Example:
    - we'll create a package named my_robot_tutorials and specify the dependencies roscpp, rospy, std_msgs.
    - These dependencies will be automatically added to the package's configuration files.
```
  $ catkin_create_pkg my_robot_tutorials roscpp rospy std_msgs
  # i choose my_robot_tutorials as the name of the package
```

- This command will generate the necessary files and directories for the new ROS package, including:
  - ``CMakeLists.txt`` : The main configuration file for the package.
  - ``package.xml`` : The package manifest file, which contains metadata about the package.
  - ``src`` directory: Where you'll place your C++ source files.
  - ``include`` directory: Where you'll place your C++ header files.

3. Verify the package creation by navigating to the catkin_ws directory and listing the contents:
```
  $ cd catkin_ws/
  $ ls
  # build  devel  src
```
```
  $ cd src/
  $ ls
  # CMakeLists.txt  my_robot_tutorials
```
```
  $ cd my_robot_tutorials/
  $ ls
  # CMakeLists.txt  include  package.xml  src
```

4. You can now open the CMakeLists.txt file to review the package's configuration:

```
  $ vim CMakeLists.txt 
```
- This file will contain the necessary instructions for building and compiling your ROS package.
  
   <img src="https://github.com/alanoudmk/ROS-Noetic-Tutorials/assets/127528672/1eb3d8d0-dc31-4740-9238-79617e8a7c8d" width="290" height="120">

***



## 5. Creating ROS Node with Python:

<mark>Node:<mark>
- The fundamental building blocks of a ROS application. A node is an executable program that runs inside the robot application and performs a specific task.
- Nodes communicate using a publish-subscribe messaging model. Nodes can publish messages to topics, and other nodes can subscribe to those topics to receive the messages.

1. Create a scripts file:
```
  $ cd catkin_ws/src/my_robot_tutorials/
  $ ls
  # CMakeLists.txt  include  package.xml  src
```
2. Create your first node in it:
 ```
  $ mkdir scripts
  $ cd scripts/
  $ ls
  $ touch my_first_node.py
```

3. Make the node executable:
```
  $ chmod +x my_first_node.py
```

4. Edit the node:
```
  $ gedit my_first_node.py
```

- Write the [code](https://github.com/alanoudmk/ROS-Noetic-Tutorials/blob/main/catkin_ws/src/my_robot_tutorials/scripts/my_first_node.py).

- click _SAVE_, Then _Exit_:

  <img src="https://github.com/alanoudmk/ROS-Noetic-Tutorials/assets/127528672/4b2ea5e5-c30b-4cac-bfd1-6f6e4832fa1f" width="450" height="130">


5. Open a new **Terminal** and run roscore: 
   > Ctrl + Alt + T

```
  $ roscore
```

6. Return to the first **Terminal**, write:

```
  $ cd ~/catkin_ws/src/my_robot_tutorials/scripts
  $ python3 my_first_node.py 
```

- You should see the following output:

  <img src="https://github.com/alanoudmk/ROS-Noetic-Tutorials/assets/127528672/9c18defe-6a5b-42dc-8a07-7f0b2e10af60" width="400" height="200">


7. View all ROS nodes that are currently running in the ROS environment:
```
  $ rosnode list
```

   <img src="https://github.com/alanoudmk/ROS-Noetic-Tutorials/assets/127528672/49b58aa2-b24b-4383-b7bf-de28df943789" width="220" height="50">

***



## 6. Creating ROS Node with C++:

1. Go inside the Source folder:
```
  $ cd catkin_ws/src/my_robot_tutorials/src/
```

2. Create your C++ file:
 ```
  $ touch my_first_node.cpp

```

3. Edit the node:
```
  $ gedit my_first_node.cpp
```
- write:
```
  #include <ros/ros.h>
  int main( int argc, char **argv){
  	ros::init(argc, argv, "my_first_cpp_node");
  	ros::NodeHandle nh;
  	ROS_INFO("Node has been started");
  	ros::Rate rate(10);
	  while( ros::ok()){
		  ROS_INFO("Hello");
		  rate.sleep();
	  }
  }
```

- click _SAVE_, Then _Exit_:

  <img src="https://github.com/alanoudmk/ROS-Noetic-Tutorials/assets/127528672/1bf97d8d-290e-4f30-8553-2a1bcb7aa0c2" width="450" height="110">

4. Make the node executable:
```
  $ cd ..
  $ gedit CMakeLists.txt 
```
- write:
```
  add_executable(node_cpp src/my_first_node.cpp)
  target_link_libraries(node_cpp ${catkin_LIBRARIES})
```
- click _SAVE_, Then _Exit_:
<img src="https://github.com/alanoudmk/ROS-Noetic-Tutorials/assets/127528672/b94a320f-727c-46ae-9cde-c11398d312dd" width="470" height="60">

  
5. Go to the Catkin ws_ directory:
```
  $ cd catkin_ws/
  ~/catkin_ws$ catkin_make
```

6. Open a new **Terminal** and run roscore: 
   > Ctrl + Alt + T

```
  $ roscore
```

7. Return to the first **Terminal**, start the node:

```
  $ cd devel/lib/my_robot_tutorials/
  $ ./node_cpp
 
```

- You should see the following output:

  <img src="https://github.com/alanoudmk/ROS-Noetic-Tutorials/assets/127528672/9f3234a9-570d-42e6-b539-726322004ed5" width="380" height="100">

  
***

## 7. Debugging Nodes with Command-Line Tools:
Debug your Nodes with Command Line Tools:


1.  Open a **Terminal** & Launch  ROS Master: 
   > Ctrl + Alt + T

 ```
    $ roscore
```
2. Start a Node using _rosrun_:
   - Open another **Terminal** and write:
   - Press _Tab_ key on your keyboardto Twice to help you autocomplete the command.
     
```
    $ rosrun my_robot_tutorials 
   # my_first_node.py  node_cpp
```

3. Run the Node:
```
	$ rosrun my_robot_tutorials node_cpp 
	# [ INFO] [1720627420.614726780]: Node has been started
	# [ INFO] [1720627420.616617024]: Hello
	# [ INFO] [1720627420.718970798]: Hello
	# [ INFO] [1720627420.818393970]: Hello
```

4. Differentiate between:
   - Executable Name
     	- actual name of the compiled script that you run to start the ROS node, the name you use with the _rosrun_ command to launch the node.
   - File Name
     	- the source code file that contains the implementation of the ROS node, e.g.  my_first_node.py and node_cpp.cpp.
   - Node Name
     	- to identify the ROS node within the ROS system, typically defined in node's source code, using rospy.init_node() or nh.advertise() functions.

5. Useful Command-Line Tools:
   - $ _rosnode list_:
     	- will show all nodes registerd in the graph.
   - $ _rosnode info_
     	- get info about one running node.
   - $ _rosnode kill_
     	- to kill a node or press _Ctrl + C_.

***



## 8. Visualize ROS Graph:

1.  Open a **Terminal** & Launch  ROS Master: 
   > Ctrl + Alt + T

 ```
    $ roscore
```

2. Open the ROS Graph Viewer:
```
	$ rosrun rqt_graph rqt_graph 
```

- remove Debug.
- You can see that there's nothing yet, just _rosout_. 

   <img src="https://github.com/alanoudmk/ROS-Noetic-Tutorials/assets/127528672/3d055f0e-a6fe-42cd-9dba-1d30d545153b" width="360" height="190">

3. Now Lets Start the Node You Built:
   - Open a **Terminal** & run:
```
	$ rosrun my_robot_tutorials node_cpp
```

4. Reload the ROS Graph Viewer:
   - remove Debug.
   - You can see that there's only one node now.
   
    <img src="https://github.com/alanoudmk/ROS-Noetic-Tutorials/assets/127528672/40a00c6c-3f96-41aa-ad6e-2e2ed38f2d8e" width="360" height="190">

5. Start Another Node::
   - Open a **Terminal** & run:
```
	$ rosrun my_robot_tutorials 
	# my_first_node.py  node_cpp          
	$ rosrun my_robot_tutorials my_first_node.py 
```


5. Reload the ROS Graph Viewer:
   - You can see that there are now two nodes.
   
    <img src="https://github.com/alanoudmk/ROS-Noetic-Tutorials/assets/127528672/c80fab81-6bea-419a-8f6d-c3974d2de2b4" width="360" height="190">


***


## 9. Experiment on Nodes with Turtelsim:
- [HERE](https://github.com/alanoudmk/ROS1-Turtlesim-Controller) the tutorials.



***

## 10. What is a ROS Topic?

- Topics are a fundamental part of the ROS communication architecture, allowing for flexible information exchange between different nodes in a ROS-based system. They enable the building of modular and scalable robot applications by allowing nodes to be developed and integrated independently.
  - Unidirectional data stream (publisher/subsciber).
  - has a Message Type.
  

***



## 11. Create ROS Publisher with Python:

1. Go to the _scripts folder_:
```
	$ cd catkin_ws/src/my_robot_tutorials/scripts/
```

2. Create a New Node:
```
	$ touch robot_news_radio_transmitter.py
```

3. Make it executable:
```
	$ chmod +x robot_news_radio_transmitter.py 
```

4. Edit the node file:
```
	$ gedit robot_news_radio_transmitter.py
```

 -  Write the [code](https://github.com/alanoudmk/ROS-Noetic-Tutorials/blob/main/catkin_ws/src/my_robot_tutorials/scripts/robot_news_radio_transmitter.py)

  - click _SAVE_, Then _Exit_:
    
5. Open a new **Terminal** and run roscore: 
   > Ctrl + Alt + T

```
	$ roscore
```

6. Launch the node:
```
	$ python3 robot_news_radio_transmitter.py 
```
     

7.  Open a new **Terminal** and run:
```
	$ rostopic list 
```

   - This will list all the topics currently available in the ROS system

     
8. Listen to the topic by command line:
```
	$ rostopic echo /robot_news_radio 
```



***


## 12. Create ROS Subscriber with Python:


1. Go to the _scripts folder_:
```
	$ cd catkin_ws/src/my_robot_tutorials/scripts/
```

2. Create a New Node:
```
	$ touch smartphone.py
```

3. Make it executable:
```
	$ chmod +x smartphone.py
```

4. Edit the node file:
```
	$ gedit smartphone.py
```

 -  Write the [code](https://github.com/alanoudmk/ROS-Noetic-Tutorials/blob/main/catkin_ws/src/my_robot_tutorials/scripts/smartphone.py)

 - click _SAVE_, Then _Exit_:
    
5. Open a new **Terminal** and run roscore: 
   > Ctrl + Alt + T

```
	$ roscore
```

6. Launch the node:
```
	$ python3 smartphone.py  
```
     

7.  Open a new **Terminal** and run the publisher node
```
	$  rosrun my_robot_tutorials robot_news_radio_transmitter.py 
```

   - From the _Subscriber_ terminal, you should see the data being published by the _Publisher_ node

<img src="https://github.com/alanoudmk/ROS-Noetic-Tutorials/assets/127528672/3c52c764-abcc-4ed0-a30c-6705291985d4" width="510" height="210">


***


## 13. Create ROS Publisher with C++:

1. Go to the _scripts folder_:
```
	$ cd catkin_ws/src/my_robot_tutorials/scripts/
```

2. Create a New Node:
```
	$ touch robot_news_radio_transmitter.cpp
```

3. Edit the node file:
```
	$ gedit robot_news_radio_transmitter.cpp
```

 -  Write the [code](https://github.com/alanoudmk/ROS-Noetic-Tutorials/blob/main/catkin_ws/src/my_robot_tutorials/scripts/robot_news_radio_transmitter.cpp).

 - click _SAVE_, Then _Exit_:

4. Make the node executable:
```
	$ cd ..
	$ gedit CMakeLists.txt 
```
- write:
  
```
	add_executable(robot_news_radio_transmitter src/robot_news_radio_transmitter.cpp)
	target_link_libraries(robot_news_radio_transmitter ${catkin_LIBRARIES})
```
- click _SAVE_, Then _Exit_:
<img src="https://github.com/alanoudmk/ROS-Noetic-Tutorials/assets/127528672/b94a320f-727c-46ae-9cde-c11398d312dd" width="470" height="60">

  
5. Go to the Catkin ws_ directory:
```
	$ cd catkin_ws/
 	 ~/catkin_ws$ catkin_make
```

6. Launch node:
```
	$ rosrun my_robot_tutorials robot_news_radio_transmitter
	$ rostopic echo /robot_news_radio
```


***


## 14. Create ROS Subscriber with C++:


***


## 15. Topic Activity:




***


## 16. What is a Service?


***


## 17. Create a Python Service Server:




***


## 18. Create a Python Service Client:


***


## 19. Create a C++ Service Server & Client:


***


## 20. 


***


## 21. 


***


## 22. 

***


## 23. 


***


## 24. 

***


## 25. 


***


## 26. 




***


## **Useful Resources**

- [Full Instructions](https://wiki.ros.org/Installation/Ubuntu) provided by the ROS organization.
- YouTube playlist on ROS Noetic available [here](https://youtu.be/Qk4vLFhvfbI?si=vQ72YrGRS629p7wb).
- Udemy step-by-step course: [ROS1 for Beginners](https://rbcknd.com/ros-for-beginners) 


