# ROS Computation Graph

## Introduction

In today's lecture, we will delve into the fundamentals of the ROS (Robot Operating System) computation graph, particularly focusing on the concept of nodes and their communication mechanisms. The ROS computation graph is a pivotal aspect of ROS, facilitating the exchange of data and commands between different parts of a robotic system.

### What is the ROS Computation Graph?

The ROS computation graph is essentially a network of interconnected nodes, where each node represents a distinct computational entity. These nodes can be written in various programming languages such as C++ or Python, enabling flexibility and versatility in development.

## Nodes in ROS Computation Graph

Nodes in the ROS computation graph perform specific tasks and communicate with each other to accomplish complex functionalities. They can publish or subscribe to topics, offer or consume services, and engage in action-based interactions.

### Publisher-Subscriber Communication

The most fundamental mode of communication in ROS is the publisher-subscriber model. In this paradigm, nodes can publish messages to a specific topic, while other nodes can subscribe to these topics to receive the messages. This asynchronous communication enables data exchange between disparate parts of the system.

### Writing a Publisher Node

Let's start by creating a publisher node in both C++ and Python to understand how messages are published in ROS.

#### C++ Publisher Node

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "cpp_publisher");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(1);

    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = "Hello, ROS!";
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
```

#### Python Publisher Node

```python
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publisher():
    rospy.init_node('python_publisher', anonymous=True)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        msg = String()
        msg.data = "Hello, ROS!"
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
```

### Subscriber Node

To receive messages published by the publisher nodes, we need to create a subscriber node.

#### C++ Subscriber Node

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cpp_subscriber");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    ros::spin();

    return 0;
}
```

#### Python Subscriber Node

```python
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard %s", data.data)

def listener():
    rospy.init_node('python_subscriber', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
```

## Conclusion

In this tutorial, we've explored the foundational concepts of the ROS computation graph, focusing on the publisher-subscriber communication model. We've created publisher and subscriber nodes in both C++ and Python to demonstrate how messages are exchanged within a ROS system. This is just the beginning of your journey into ROS, where you'll continue to explore more advanced communication mechanisms like ROS Services and ActionLib. Happy coding!