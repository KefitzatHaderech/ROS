# Master Node

ROS (Robot Operating System) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of platforms. This tutorial will guide you through the initial steps of setting up and understanding ROS, focusing on starting the ROS core, identifying nodes, and exploring topics.

1. Setting Up ROS Core:
   - To begin working with ROS, the first step is to start the ROS core. The ROS core is essentially the central hub of communication for all ROS nodes.
   - Open a terminal window and type the following command:

     ```
     roscore
     ```

   - Upon successful execution, you will see output similar to the following:

     ```
     ... logging to /home/user/.ros/log/0c5b4ef8-a2e4-11eb-8c25-0242ac130003/roslaunch-yourcomputername-12345.log
     Checking log directory for disk usage. This may take a while.
     Press Ctrl-C to interrupt
     Done checking log file disk usage. Usage is <1GB.
     
     started roslaunch server http://yourcomputername:12345/
     ros_comm version 1.15.10
    
     SUMMARY
     ========
    
     PARAMETERS
      * /rosdistro: noetic
      * /rosversion: 1.15.10
    
     NODES
    
     auto-starting new master
     process[master]: started with pid [1234]
     ROS_MASTER_URI=http://localhost:11311/
     
     setting /run_id to 0c5b4ef8-a2e4-11eb-8c25-0242ac130003
     process[rosout-1]: started with pid [1235]
     started core service [/rosout]
     ```

   - Important parameters to note:
     - ROS distribution (e.g., "noetic"): Indicates the version of ROS being used.
     - ROS master URI: Specifies the URI of the ROS master server (usually running on localhost and port 11311).

2. Identifying Nodes:
   - Nodes in ROS are individual processes that perform computation. After starting the ROS core, you can identify running nodes using the following command:

     ```
     rosnode list
     ```

   - Since only the ROS core (rosout) is running at this stage, the output will display only one node.

3. Exploring Topics:
   - Topics in ROS are named buses over which nodes exchange messages. To view the list of available topics, use the following command:

     ```
     rostopic list
     ```

   - Initially, you will see a limited number of topics related to the ROS core.

Conclusion:
Congratulations! You have successfully started the ROS core, identified nodes, and explored topics within the ROS ecosystem. This tutorial provides a foundational understanding of how ROS operates and sets the stage for further exploration and development in robotics. Experiment with creating your own nodes, publishing and subscribing to topics, and leveraging ROS's powerful features to build sophisticated robot applications.
