# ROS: Basic, Motion, and OpenCV

## 1. ROS Software Development

1. Introduction
2. Why decide to use ROS?
3. The Robot Life Cycle
4. Self-Driving Car Use Case
5. ROS Evolution
6. ROS Distributions
7. ROS Architecture
8. ROS Basic Concepts
9. ROS Communication Paradigms
10. ROS Path Planning and Navigation
11. Limitation of ROS
12. ROS2

## 2. Setting your environment with ROS Noetic

1. Install ROS Noetic on Ubuntu 20.04
2. Create your catkin workspace with ROS Noetic
3. Clone the Repository for the ROS Noetic distribution
4. Testing your installation with C++ nodes
5. Testing your installation with Python nodes

## 3. Create a ROS-Workspace and a ROS-Package

1. Section Note Update
2. ROS Workspace and ROS Package
3. Wrapping up

## 4. ROS Computation Graph

1. What is a ROS Computation Graph?
2. ROS Computation Graph Life Cycle
3. Start the ROS Master Node
4. How to run a new node (executable) in ROS?
5. What happens when we start a new ROS node?
6. Adding a teleop node to make the robot move
7. Get the information of a node and the information of a topic
8. The content of the motion message /tutle1/cmd_vel
9. Understand the structure of a ROS message
10. How to show the message structure on ROS command line?
11. Publish a message on a topic from a command line
12. Visualize the ROS Computation Graph using ros_rqt_graph
13. Demo: Starting Turtlesim and checking information about the nodes and topics
14. Demo: Showing the content of ROS messages published
15. Demo: Understand the pose topic
16. Demo: What is the benefit of using ROS?
17. Demo: Publishing a message from a command line using rostopic pub
18. Demo: rqt_graph

## 5. ROS Topics

1. ROS Topics Overview
2. Question: what happens if ROS Master crashes?
3. Guidelines to Write a Publisher and a Subscriber in ROS
4. Overview of the Talker/Listener Application (ROS Hello World Example)
5. Write a Publisher Node in Python
6. Write a Subscriber Node in Python
7. Talker/Listener in Python
8. Write a Publisher/Subscriber Node in C++
9. Talker/Listener in C++
10. Do-It-Yourself Assignment Explanation


## 6. ROS Messages

1. Create Custom ROS Messages: Overview
2. Create a Custom ROS Message: Implementation
3. IoTSensor Custom Message Publisher/Subscriber Applications

## 7. ROS Services

1. What is a ROS Service?
2. Understand ROS Services with Turtlesim
3. ROS Services with Turtlesim
4. AddTwoInts Service Overview
5. Create the Service File and Request/Response Messages
6. Write ROS Service (Client/Server) in Python
7. Write ROS Service (Client/Server) in Python
8. Writing a ROS Service (Client/Service) in C++

## 8. Motion in ROS

1. Note about this updated section
2. Cleaning Application Overview
3. Step 1. Understand Topics and Messages Used in the Application
4. Import the Libraries in Python and C++
5. The Divide and Conquer Approach
6. Move Straight Line (C++/Python)
7. Move Straight Line (C++/Python)
8. Rotate Motion (C++/Python)
9. Rotate Motion (C++/Python)
10. Go-To-Goal Behavior
11. Go-To-Goal Behavior
12. Set Desired Orientation
13. Set Desired Orientation
14. Spiral Trajectory
15. Spiral Trajectory Demo
16. Putting All Together: The Cleaning Application
17. Running Multiple Nodes with a Launch File

## 9. ROS Tools and Utilities

1. ROS Network Configuration
2. Launch Files (Part I): Running multiple nodes with roslaunch
3. Launch Files (Part II): including a launch file and define parameters

## 10. Turtlebot 3

1. Install Turtlebot3 Simulator
2. Overview of Turtlebot3 Simulation Environment
3. Simple Demo with Turtlebot3

## 11. Computer Vision in ROS with OpenCV

1. OpenCV Overview
2. Install OpenCV for ROS (Melodic/Kinetic)
3. Install OpenCV for ROS Noetic
4. Additional Note about OpenCV installation
5. OpenCV: Open/Save Image Files (Python)
6. OpenCV: Pixels and Image Structure (Python)
7. OpenCV: Image Encoding (Python)
8. OpenCV: Video Streams Input (Python)
9. OpenCV: Drawing Shapes
10. CvBridge: Bridging OpenCV and ROS
11. Simple and Adaptive Thresholding in OpenCV
12. Color Filtering in OpenCV (Tennis Ball use case)
13. Contours Detection and Processing in OpenCV
14. Tennis Ball Detection using OpenCV
15. OpenCV (C++) Video Input/Output
16. OpenCV (C++) Open and Save Images
17. OpenCV (C++) CvBridge: Bridging Images between OpenCV and ROS

## 12. Laser Range Finders

1. What is a Laser Range Finder?
2. Laser Range Finders Characteristics
3. Commercial Laser Range Finder Devices
4. Laser Scanner Devices used in Demonstrations
5. Connect Asus Live Pro RGBD Camera as a Laser Scanner
6. Connect Hokuyo URG 4LX Laser Scanner with ROS
7. Save Laser Scan Messages into a Bag File and Replay Them
8. Write a ROS Node (Python) as a Subscriber to a Laser Scanner
9. Write a ROS Node (C++) as a Subscriber to a Laser Scanner

## 13. rosserial: Connecting a Hardware with ROS

1. What is rosserial?
2. What is Arduino?
3. Arduino Uno Board
4. Arduino IDE Overview
5. Install and Setup Arduino IDE
6. Range Sensor and Connection Setup
7. Programming The Ultrasonic Sensor with Arduino
8. Install rosserial libraries
9. rosserial Hello World application: Arduino ROS Publisher
10. rosserial Blink application: Arduino ROS Subscriber
11. rosserial using Arduino Ultrasonic Range Sensor
