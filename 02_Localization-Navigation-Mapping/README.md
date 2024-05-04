# ROS: Localization, Navigation and SLAM




## 1. Overview

1. [What is this course about?](#)
2. [About the instructor](#)
3. [What to do when you face errors and need to debug and find solutions?](#)
4. [Important: Install ROS Navigation and SLAM](#)
5. [Important: Installation Instructions of TB3](#)
6. Install Turtlebot3 Simulator
7. Turtlebot3 Simulation Environments
8. Simple Turtlebot3 Demo
9. Get the code from GitHub Repository
10. Note about path setting for map-based navigation
11. Link to the pre-installed virtual machine

## 2. ROS Navigation Demo

12. Navigation Demo Intro
13. Important note about the demo
14. Start the Turtlebot3 Simulator
15. If you have a problem starting the Turtlebot3 simulator
16. Setting the initial location of the robot
17. Frames
18. Location of the robot in different frames
19. How orientation is represented in 3D space?
20. Navigation Example

**Assignment 1:** General Question on the Navigation Demo

## 3. 2D Frames, Transformations, and Localization

21. Section Overview and Learning Outcomes
22. Pose of a robot in a frame
23. Dealing with Multiple Frames
24. Transformations Overview
25. 2D Translation
26. 2D Rotation
27. The General Transformation Matrix: Translation + Rotation
28. Illustrative Example on 2D Transformation

**Quiz 2:** 2D Transformations

## 4. 3D Frames, Transformations, and Localization

29. 3D Frame Introduction
30. The Right-Hand Rule
31. 3D Frames and Transformations
32. 3D Transformation Example

## 5. Orientation in 3D Space

33. How orientation is represented in 3D Space?
34. The Three-Angle Representation
35. The Arbitrary Vector Representation
36. Quaternions

## 6. TF Package: Frames, Transformations, and Localization in ROS

37. The tf package overview
38. What is TF in ROS?
39. URDF: Language for the Description of Frames and Transformation in a Robot Model
40. Why TF is important?
41. Overview of tf package utilities
42. Convert Orientation between Quaternion and Roll-Pitch-Yaw using TF
43. Reading the Yaw of a Robot from its Orientation in Quaternion
44. The tf package command line and utilities
45. [view_frames command line](#)
46. [Note about view_frames](#)
47. [More about view_frames](#)
48. tf_echo command line
49. tf_monitor command line
50. Static transform publisher
51. Broadcast a transformation in a ROS Node
52. Listen to a transformation in a ROS Node

## 7. Map-Based Navigation

53. Mobile Robot Navigation Overview
54. Map-based Navigation Overview
55. SLAM Demonstration and Discussions
56. Understand the structure of Maps in ROS
57. Understand ROS Nodes and Launch File used for SLAM
58. Map-based Navigation Demo
59. The Recovery Behavior
60. Understand the navigation launch file
61. Writing a ROS Node for Robot Navigation (Python)
62. Understand the Recovery Behaviors of the Navigation Stack
63. Robot Setup to Support the ROS Navigation Stack

## 8. Configuration and Tuning of the Navigation Stack Parameters

64. Navigation Stack Tuning: Problem Statement
65. Tuning Max/Min Velocities and Accelerations of the Robot
66. Global Planner Parameter Tuning
67. Local Path Planner Overview
68. The Dynamic Window Approach (DWA) Algorithm
69. Tuning the Simulation Time of the DWA Algorithm
70. DWA Trajectory Scoring
71. Tuning the DWA Trajectory Scores

## 9. Reactive Navigation

72. Overview
73. Overview of the Follower Application
74. Create a TF Broadcaster for the frames attached to the robot
75. TF Listener for the follower
76. Bug Algorithms: Overview
77. BUG0, BUG1, and BUG2 Approaches
