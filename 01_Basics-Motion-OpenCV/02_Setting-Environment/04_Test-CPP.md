# Testing ROS Essential CP Package Installation

**Introduction:**
In this tutorial, we will guide you through testing the installation of the ROS (Robot Operating System) Essential CP package. We will ensure that your ROS environment is set up correctly by running two example nodes: the Talker and the Listener. This will confirm that your ROS installation and the Essential CP package are functioning properly.

**Prerequisites:**

1. Basic understanding of ROS concepts.
2. ROS installed on your system.
3. Familiarity with using terminals in a Linux environment.

**Steps to Test ROS Essential CP Package Installation:**

**Step 1: Verify ROS Installation:**
Ensure that ROS is correctly installed on your system. You can verify this by running the following command in your terminal:

```bash
roscore
```

If ROS is installed properly, you should see the ROS Master start and display information about the ROS environment.

**Step 2: Navigate to Essential CP Package Examples:**
Locate the Essential CP package examples directory in your ROS workspace. This directory typically contains demo examples provided with the package.

**Step 3: Run the Talker Node:**
a. Open a new terminal window.
b. Navigate to the Essential CP package examples directory.
c. Run the Talker node using the following command:

```bash
rosrun <package_name> talker
```

Replace `<package_name>` with the name of the package containing the Talker node, usually something like `essential_cp`.

**Step 4: Verify Talker Node Operation:**
Once the Talker node is running, it will start publishing "Hello World" messages. Ensure that you see these messages in the terminal where you launched the Talker node. This confirms that the Talker node is functioning correctly.

**Step 5: Run the Listener Node:**
a. Open another terminal window.
b. Navigate to the Essential CP package examples directory.
c. Run the Listener node using the following command:

```bash
rosrun <package_name> listener_node
```

Replace `<package_name>` with the name of the package containing the Listener node.

**Step 6: Verify Communication Between Nodes:**
After launching the Listener node, it will subscribe to the topic published by the Talker node. Check the terminal where you launched the Listener node for any incoming messages. If you see the "Hello World" messages being received, it indicates that communication between the Talker and Listener nodes is successful.

**Step 7: Confirmation:**
If you have successfully completed Steps 1 to 6 without encountering any errors, congratulations! Your ROS installation and the ROS Essential CP package are set up correctly.
