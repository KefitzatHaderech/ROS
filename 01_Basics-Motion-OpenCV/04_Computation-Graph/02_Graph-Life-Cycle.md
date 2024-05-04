# Understanding ROS Computation Graph with Turtle Robot Example

Introduction:
Robot Operating System (ROS) is a powerful framework for building robotic applications. One of its key features is the ROS computation graph, which facilitates communication between different nodes in a robotic system. In this tutorial, we will delve into the intricacies of ROS computation graph using a specific example involving a turtle robot. We will explore how nodes interact with each other through topics, publishers, subscribers, and the ROS master.

Prerequisites:
Before diving into this tutorial, it's recommended to have a basic understanding of ROS concepts such as nodes, topics, publishers, and subscribers. Additionally, familiarity with the command line interface and ROS commands will be beneficial.

Example Scenario:
Imagine we have a turtle robot operating within a ROS environment. We'll walk through the steps involved in setting up and understanding the ROS computation graph for this scenario.

1. ROS Computation Graph Overview:
   - The ROS computation graph consists of nodes, which are individual processes that perform computation.
   - Nodes communicate with each other by publishing messages to topics and subscribing to topics to receive messages.
   - The ROS master is a crucial component that facilitates communication between nodes by managing the registration of publishers and subscribers.

2. Starting the ROS Core (ROS Master):
   - Before any communication can occur, we need to start the ROS core, also known as the ROS master.
   - Open a terminal and run the following command:
     ```
     roscore
     ```
   - This command initializes the ROS master, allowing nodes to register and communicate with each other.

3. Starting the Turtle Robot Node:
   - The turtle robot node represents our robot within the ROS environment.
   - In a new terminal, start the turtle node:
     ```
     rosrun turtlesim turtlesim_node
     ```
   - This command launches the turtle simulation node, which simulates the movement of our robot.

4. Announcing the Turtle Node to ROS Master:
   - Once the turtle node is running, it needs to announce itself to the ROS master.
   - The turtle node registers itself with the ROS master, specifying its name as "TurtleSIM" and the topics it will publish and subscribe to.

5. Publisher Node Controlling the Robot:
   - Now, let's create another node that controls the turtle robot's movement.
   - Open a new terminal and create a publisher node:
     ```
     rosrun <publisher_package> <publisher_node>
     ```
   - Replace `<publisher_package>` and `<publisher_node>` with appropriate package and node names for your publisher.

6. Publishing Messages to CMD Topic:
   - The publisher node publishes messages to a specific topic, such as the "cmd" topic in our example.
   - These messages contain commands for controlling the turtle robot's movement.

7. Subscribing Node Receiving Robot Commands:
   - Alongside the publisher node, we need a subscriber node that receives the commands published by the publisher.
   - Open another terminal and create a subscriber node:
     ```
     rosrun <subscriber_package> <subscriber_node>
     ```
   - Replace `<subscriber_package>` and `<subscriber_node>` with appropriate package and node names for your subscriber.

8. ROS Master Facilitating Communication:
   - As nodes are created and messages are published and subscribed to, the ROS master plays a vital role in facilitating communication.
   - It keeps track of publishers and subscribers, ensuring that messages are correctly routed between them.

9. Message Exchange between Publisher and Subscriber:
   - With the publisher and subscriber nodes running and registered with the ROS master, communication can occur.
   - The publisher publishes messages containing commands for the turtle robot's movement.
   - The subscriber receives these messages and processes them accordingly, controlling the robot's behavior.

Conclusion:
In this tutorial, we explored the ROS computation graph using a turtle robot example. We learned how nodes interact with each other through topics, publishers, subscribers, and the ROS master. Understanding these concepts is essential for developing complex robotic systems within the ROS framework. Experiment with different nodes, topics, and message types to gain a deeper understanding of ROS communication.