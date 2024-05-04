
# ROS: Web-Based Navigation with ROS-Bridge

## 1. Introduction

1.1 [Introduction](#introduction)
1.2 [Overview of Libraries and Resources](#overview-of-libraries-and-resources)
1.3 [Important - Seed project structure](#)
1.4 [How to follow this course?](#how-to-follow-this-course)
1.5 [What if you face errors and need to debug and find solutions?]()
1.6 [Accessing the Instructor Code for the Udemy ROSBridge Course](#accessing-the-instructor-code-for-the-udemy-rosbridge-course)

## 2. Create a Basic React Application

2.1 [Install a new React Application](#install-a-new-react-application)
2.2 [Understand the anatomy of a React App](#understand-the-anatomy-of-a-react-app)
2.3 [Setting-up the bootstrap theme](#setting-up-the-bootstrap-theme)
2.4 [Create Header and Footer components](#create-header-and-footer-components)
2.5 [Add a navigation bar to the header component](#add-a-navigation-bar-to-the-header-component)
2.6 [Create a Router of Pages](#create-a-router-of-pages)
2.7 [Customize the footer](#customize-the-footer)

## 3. Connection Component: Establish a Robot Connection with the Web App

3.1 [Overview of the `<Connection/>` component](#overview-of-the-connection-component)
3.2 [Understand the concept of state in React](#understand-the-concept-of-state-in-react)
3.3 [Create the `<Connection/>` Component](#create-the-connection-component)
3.4 [Import the ROSLIB Dependency](#import-the-roslib-dependency)
3.5 [Create an Alert component](#create-an-alert-component)
3.6 [Connection Logic with ROSLib and React State](#connection-logic-with-roslib-and-react-state)
3.7 [Connect to ROSBridge](#connect-to-rosbridge)
3.8 [Dealing with automatic reconnection](#dealing-with-automatic-reconnection)
3.9 [Clean code and use configuration file](#clean-code-and-use-configuration-file)

## 4. Teleoperation Component: Drive the Robot from the Web App

4.1 [Create a Teleoperation Component](#create-a-teleoperation-component)
4.2 [Arrange page layout using the React Grid structure](#arrange-page-layout-using-the-react-grid-structure)
4.3 [Develop the React Joystick Component](#develop-the-react-joystick-component)
4.4 [The structure of the handleMove method](#the-structure-of-the-handlemove-method)
4.5 [Create a cmd_vel publisher inside handleMove](#create-a-cmd_vel-publisher-inside-handlemove)
4.6 [Create and publish a twist message using ROSLib](#create-and-publish-a-twist-message-using-roslib)
4.7 [The event property of the react joystick component](#the-event-property-of-the-react-joystick-component)
4.8 [Setting the right velocities from the Joystick component throttle](#setting-the-right-velocities-from-the-joystick-component-throttle)
4.9 [The HandleStop Method](#the-handlestop-method)

## 5. Display Robot Position, Orientation and Speed

5.1 [Note before watching next lectures](#note-before-watching-next-lectures)
5.2 [Setup the Turtlebot3 Simulator](#setup-the-turtlebot3-simulator)
5.3 [Test Teleop with Turtlebot3](#test-teleop-with-turtlebot3)
5.4 [Create the Robot State ReactJS Component](#create-the-robot-state-reactjs-component)
5.5 [Create a position subscriber and visualize position on the map](#create-a-position-subscriber-and-visualize-position-on-the-map)
5.6 [Create a subscriber for the orientation](#create-a-subscriber-for-the-orientation)
5.7 [Create a subscriber for the velocities](#create-a-subscriber-for-the-velocities)

## 6. Map-Based Navigation on the Browser

6.1 [Overview](#overview)
6.2 [Create a `<Map>` component](#create-a-map-component)
6.3 [Initialize a ROS Connection in `<Map>` Component](#initialize-a-ros-connection-in-map-component)
6.4 [Importing Map and Navigation JS Libraries](#importing-map-and-navigation-js-libraries)
6.5 [Visualize the map on the browser](#visualize-the-map-on-the-browser)
6.6 [Improve the page layout](#improve-the-page-layout)
6.7 [Visualize the robot position on the map](#visualize-the-robot-position-on-the-map)
