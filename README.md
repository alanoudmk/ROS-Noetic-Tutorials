# What is ROS 1 Noetic ?
- ROS (Robot Operating System) Noetic is the 14th major release of the ROS software framework, released in May 2020. It is a long-term support (LTS) release, with extended maintenance and support until April 2025.

- We will be working on:
  - Ubuntu 20.04.6 LTS (Focal Fossa).
  - ROS 20.04.
  - Windows 10 and above.

## 1. Install and Setup ROS Noeatic:
- [HERE](https://github.com/alanoudmk/Install-ROS-Noetic-on-Ubuntu) is how to download it.



***

## 2. Launch Your First ROS Master

1- Open a **Terminal**: 
   > Ctrl + Alt + T

2- Launch  ROS Master:
- The ROS Master is the central point of the ROS system, responsible for managing communication between ROS nodes.
- By launching roscore, you are starting the ROS Master, which allows other ROS nodes to register with it and exchange data and commands.
```
$ roscore
```

3- limitations:
- The ROS Master can only run as a single instance on a system at a time.
- Attempting to run multiple instances of roscore simultaneously will result in errors, as each ROS Master requires exclusive access to the ROS communication infrastructure.
- Single Point of Failure


***



## 3. Setting Up Your First ROS Program (Catkin Workplace)
- Catkin is the official build system used by the Robot Operating System (ROS). 
  - it empowers ROS developers to optimize their workflow by simplifying the process of building, testing, and deploying ROS-based projects. This streamlined approach makes it easier for developers to manage and share their work within the ROS community.
    
1- Open a **Terminal**: 
   > Ctrl + Alt + T

2- Create the Workspace Directory: 
```
$ cd
$ mkir catkin_ws
```

3- Create The Source Folder: 
```
$ cd catkin_ws/
$ mkdir src
$ catkin_make
```

4- You should see the following output:

<img src="https://github.com/alanoudmk/ROS-Noetic-Tutorials/assets/127528672/326d80bf-42e4-4c63-8681-0063294c4aa9" width="290" height="60">


5- To ensure the build was successful, run:
```
$ ls
```

<img src="https://github.com/alanoudmk/ROS-Noetic-Tutorials/assets/127528672/60ca20fe-cce6-4b95-b0b8-61af067d8215" width="240" height="40">

***



**


## 4.



***

## 5.


***



## 6.




***

## 7.


***



## 8.



**


## 9.



***

## 10.


***



## 11.



**


## 12.


**


## 13.

