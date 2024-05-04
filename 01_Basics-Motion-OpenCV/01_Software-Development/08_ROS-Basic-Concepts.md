# ROS Basic Concepts

## Introduction to ROS

ROS, or Robot Operating System, is a flexible framework for writing robot software. It provides tools, libraries, and conventions that help developers build complex robotic systems. In this tutorial, we'll cover the main concepts of ROS, including starting the ROS core, creating nodes, advertising topics, and communicating between nodes. We'll also demonstrate these concepts using a simple example with TurtleSim, a built-in ROS simulator.

### Prerequisites

Before starting this tutorial, make sure you have ROS installed on your system. You can follow the official ROS installation guide for your specific platform (http://wiki.ros.org/ROS/Installation).

## Understanding ROS Concepts

### Nodes

In ROS, a node is a process that performs computation. Nodes communicate with each other using topics, services, and actions. Each node can publish or subscribe to multiple topics, enabling communication between different parts of the robot system.

### Topics

Topics are named buses over which nodes exchange messages. Nodes can publish messages to a topic or subscribe to receive messages from a topic. Messages are typed data structures defined in ROS, such as sensor readings, motor commands, or any other data relevant to the robot's operation.

### ROS Master

The ROS Master is a crucial component of ROS. It provides registration and lookup services for nodes. When a node starts, it registers with the ROS Master, which keeps track of all active nodes and the topics they are publishing or subscribing to. The ROS Master facilitates communication between nodes by providing information about available topics and their publishers and subscribers.

### Messages

Messages define the structure of data exchanged between nodes. ROS provides a variety of predefined message types for common data, such as sensor data, control commands, and more. Users can also define custom message types for specific applications.

## Running a ROS Application

Now, let's walk through the process of running a simple ROS application using TurtleSim and teleoperation nodes as examples.

1. **Start ROS Core (ROS Master):**
   Before running any ROS nodes, start the ROS Master using the `roscore` command in a terminal. The ROS Master must be running for nodes to communicate with each other.

   ```bash
   $ roscore
   ```

2. **Run TurtleSim Node:**
   Launch TurtleSim, a simple robot simulator, by executing the `rosrun` command with the package name (`turtlesim`) and node name (`turtlesim_node`).

   ```bash
   $ rosrun turtlesim turtlesim_node
   ```

3. **Run Teleoperation Node:**
   Start the teleoperation node, which allows you to send commands to control the TurtleSim robot's movement.

   ```bash
   $ rosrun <your_teleop_package> <teleop_node_name>
   ```

4. **Advertise Nodes to ROS Master:**
   Each node advertises itself to the ROS Master upon startup. The TurtleSim node advertises as a subscriber to velocity messages, while the teleoperation node advertises as a publisher of velocity commands.

5. **ROS Master Connection:**
   The ROS Master establishes connections between nodes based on their advertised topics. It enables direct communication between the TurtleSim and teleoperation nodes, facilitating message exchange.

6. **Verify Node Status:**
   Use ROS command-line tools to verify the status of running nodes and available topics. For example, to list active nodes:

   ```bash
   $ rosnode list
   ```

   To list available topics:

   ```bash
   $ rostopic list
   ```

7. **Monitor Messages:**
   You can monitor messages flowing through topics using the `rostopic echo` command. This allows you to visualize data being exchanged between nodes.

   ```bash
   $ rostopic echo <topic_name>
   ```
