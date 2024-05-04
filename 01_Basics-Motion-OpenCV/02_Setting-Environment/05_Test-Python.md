# Testing Python Code Execution in ROS

**Introduction:**
In this tutorial, we will focus on ensuring the correct execution of Python code within the Robot Operating System (ROS) environment. Specifically, we will verify the functionality using `rosrun`, a command-line tool for running ROS nodes, and examine how to resolve potential issues related to Python version compatibility.

**Prerequisites:**
Before proceeding, ensure you have the following prerequisites:

1. ROS installed on your system.
2. Basic understanding of ROS concepts and terminology.
3. Access to a terminal on a Linux-based system.

**Steps:**

**Step 1: Verify ROS Installation:**
Ensure that ROS is correctly installed on your system. You can verify this by opening a terminal and running the following command:

```bash
roscore
```

If ROS is installed properly, you should see the ROS master node initializing without any errors.

**Step 2: Execute Python Code Using `rosrun`:**
Now, let's test the execution of Python code using the `rosrun` command. We will use a basic Python script as an example. Suppose we have a ROS package named `my_package`, and within it, there is a Python script named `talker.py` responsible for publishing messages.

To execute the `talker.py` node, navigate to the package directory containing the script in a terminal and use the following command:

```bash
rosrun my_package talker.py
```

If the Python code executes correctly, you should see the node publishing messages as expected. However, if Python is not recognized or if there are compatibility issues, you may encounter errors.

**Step 3: Resolve Python Version Compatibility:**
If you encounter errors related to Python version compatibility, it's crucial to ensure that the correct Python version is being used. By default, ROS may use Python 2, whereas your code might require Python 3.

To resolve this, follow these steps:

1. Install Python 3 if it's not already installed on your system:

```bash
sudo apt install python3
```

2. Once Python 3 is installed, you can create a symbolic link to map the `python` command to Python 3:

```bash
sudo ln -s /usr/bin/python3 /usr/bin/python
```

This mapping ensures that when you type `python` in the terminal, it refers to Python 3.

**Step 4: Test Python Execution Again:**
After resolving the Python version compatibility, test the execution of the Python node again using `rosrun`. Navigate to the package directory containing the Python script and run the `rosrun` command as before:

```bash
rosrun my_package talker.py
```

If everything is set up correctly, the Python node should execute without errors, and you should see the desired functionality, such as publishing messages.

**Step 5: Verify Listener Node (Optional):**
If your ROS package includes a listener node, you can also test its functionality using `rosrun`. Ensure that the listener node is built and executable, then run it using `rosrun` in a separate terminal window:

```bash
rosrun my_package listener
```

The listener node should now be able to receive and process messages published by the `talker.py` node.
