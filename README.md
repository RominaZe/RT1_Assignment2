RT1 Asignment 2 
==================
For the assigned task, we were presented with a specialized environment that necessitated the implementation of diverse nodes, drawing upon the knowledge and skills cultivated throughout the Research Track course. This task involved the creation and integration of distinct nodes, each tailored to fulfill specific functionalities within the given environment. The challenge called for a synthesis of theoretical insights and practical expertise gained during the course, fostering a comprehensive application of research-oriented concepts. The implementation process served as a dynamic platform for the application of theoretical principles to real-world scenarios, showcasing a synthesis of academic understanding and hands-on problem-solving capabilities

# ROS Simulation Environment

## Overview

This ROS simulation environment integrates two powerful tools: Rviz and Gazebo. Each tool serves a specific purpose in enhancing the visualization and control capabilities within the simulation.

### Rviz - ROS Visualization Tool

[Rviz](http://wiki.ros.org/rviz) is a 3-dimensional visualization tool for ROS, designed to provide insights into the robot's activities, sensor inputs, and planned actions. Key features include:

- **Robot Model Visualization:** View the simulated robot model.
- **Sensor Information Logging:** Log sensor information from the robot's sensors.
- **Sensor Data Replay:** Replay logged sensor information for debugging purposes.

By visualizing what the robot perceives and plans, users can effectively debug robot applications, from sensor inputs to the execution of planned actions.

### Gazebo - ROS 3D Simulator

[Gazebo](http://gazebosim.org/) serves as the 3D simulator for ROS, providing a realistic simulation environment. Key functionalities include:

- **Robot Control using ROS Topics:** Control the robot using ROS topics, such as `/cmd_vel`.
- **Simulation Control:** Simulate the robot's movements and interactions in a 3D environment.

To facilitate robot control, the [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) tool is recommended. Launch it using:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
# The Goal of the Assignment 
For the designated task, the objective was to create a new package encompassing three distinctive nodes, each designed to fulfill specific functionalities. The breakdown of the nodes is as follows:

* Action Client Node:
This node was developed to implement an action client, enabling users to set or cancel a target with specified coordinates (x, y). A key feature is the utilization of feedback/status from the action server to determine the accomplishment of the target. Additionally, this node publishes the robot's position and velocity as a custom message (x, y, linear_x, angular_z), leveraging the values available on the topic /odom.

* Service Node (Coordinates Retrieval):
This node operates as a service node that, upon invocation, returns the coordinates of the last target set by the user. It serves as a valuable component for retrieving essential information about the robot's previous targets.

* Another Service Node (Distance and Speed Calculation):
The third node is a service node that subscribes to the robot's position and velocity using a custom message. It implements a server to compute the distance between the robot and the target, along with the robot's average speed. This node enhances the overall system by providing insightful metrics regarding the robot's spatial relationship and speed dynamics.

Furthermore, a launch file has been crafted to orchestrate the entire simulation. Notably, a parameter has been incorporated in the launch file, offering flexibility to adjust the size of the averaging window specifically for the functionalities of node (c). This parameter serves as a valuable tuning mechanism to optimize the behavior of the system.

For launching this files use 
```bash
roslaunch assignment_2_2023 assignment1.launch
```

In summary, the developed package and its constituent nodes seamlessly integrate into a cohesive simulation environment, offering a robust set of capabilities for target manipulation, information retrieval, and real-time metric computation.

# Node Explanation
-------------------
### `Node A` - Communication and Navigation Node

The primary goal of `node_a` is to manage communication and navigation within the ROS environment. It provides the following key functionalities:

1. **Access Robot's Odometry Data:**
   - Gathers odometry data to estimate changes in position over time.

2. **Publish Robot's Position and Velocity:**
   - Publishes the robot's current position and velocity as a custom message (x, y, linear_vel, angular_vel) based on the odometry data. this is publish on the topic `/position_velocity`

3. **Active Interaction with Server:**
   - Actively interacts with the server environment, initiating and potentially canceling goals dynamically.

4. **Goal Assignment and Cancellation:**
   - Allows users to input new target coordinates (x, y) to initiate a goal for the robot.
   - Provides flexibility by enabling users to cancel goals during execution.

5. **Obstacle Handling:**
   - Demonstrates flexibility in robot behavior by navigating around obstacles during execution.
   - In case of encountering obstacles, the robot continues its movement until finding a free space, then resumes progress toward the original goal.

#### Service Call Example
For optimal usage, it is recommended to restart the node to access the interface and set new goals.
```bash
rosrun assignment_2_2023 node_a
```
### Pseudo-code of node A
```bash
Initialize ROS node "node_a"

Create ROS publisher "pub" for custom message type assignment_2_2023/RobotPV
Create ROS subscriber "sub" for topic "/odom" with CallbackPositionVelocity as callback function

Define CallbackPositionVelocity(message):
    Extract position and velocity information from the received odometry message
    Create a custom message assignment_2_2023/RobotPV with the extracted data
    Publish the custom message to topic "/position_velocity"

Define ClientGoal():
    Create an action client for PlanningAction with server "/reaching_goal"
    Wait for the action server to start

    Loop:
        Get the current goal coordinates from parameters "/des_pos_x" and "/des_pos_y"
        Create a PlanningGoal with the current goal coordinates
        Send the goal to the action server

        Print the current goal coordinates
        Prompt the user for new goal coordinates (input_x, input_y)

        Set new goal coordinates in parameters "/des_pos_x" and "/des_pos_y"
        Update the PlanningGoal with the new coordinates
        Resend the updated goal to the action server

        While the goal is pending:
            Prompt the user to cancel the goal (y/n)
            If user chooses to cancel:
                Cancel the goal and print a warning
                Break out of the loop
            If user chooses not to cancel:
                Wait for the result with a timeout of 120 seconds
                If the goal is finished:
                    Print a success message
                Else:
                    Print a warning about not reaching the goal within 120 seconds
                    Break out of the loop

        Allow ROS to process any pending callbacks

Create a thread for ClientGoal

Start the ROS event loop (ros::spin())

Wait for the ClientGoal thread to finish

Exit
```


### `Node B` - Goal Reporting Node

The primary goal of `node_b` is to report the last point where `node_a` arrived. Key features include:

1. **Goal Reporting:**
   - Prints the coordinates of the last point where `node_a` arrived.
   - Even if `node_a` didn't reach the goal, `node_b` reports the coordinates set during the operation of `node_a`.

2. **Non-Interruptive Behavior:**
   - Demonstrates the ability to operate without interrupting the current action of `node_a`.
   - Ensures that results are consistently obtained, showcasing the robot's flexibility.

#### Service Call Example

```bash
rosservice call /last_goal
```

  
### `node_c` - Metrics Calculation Service Node

The primary goal of `node_c` is to implement a service that provides valuable metrics related to the robot's navigation. This node enhances the situational awareness of the robot by offering crucial information about its movement. Key features include:

1. **Distance and Average Speed Service:**
   - Implements a service named `/distance_averageVelocity` that, when called, returns:
      - **Distance:** The Euclidean distance from the current robot position to the last goal set by `node_a`.
      - **Average Linear Velocity:** The average linear velocity of the robot during its navigation.
      - **Average Angular Velocity:** The average angular velocity of the robot during its navigation

2. **Service Call:**
   - Users can call the `/distance_averageVelocity` service to obtain real-time metrics regarding the robot's performance.

3. **Dynamic Calculation:**
   - The service dynamically calculates the distance and average speed based on the robot's odometry data and the goals set by `node_a`.

4. **Integration with `node_a`:**
   - Works seamlessly with `node_a`, complementing its functionalities and providing additional insights for a comprehensive understanding of the robot's behavior.

#### Service Call Example

```bash
rosservice call /distance_averageVelocity
```

## Possible improvements to consider:
--------
1. **Increase Robot Speed:**
   - Elevate the robot's speed for efficient target attainment by refining velocity parameters or implementing a dynamic control mechanism. Consider integrating a service that, upon activation, sends a command to `cmd_vel` to seamlessly adjust the robot's velocity, allowing for both incremental increases and decreases as needed. 

2. **Add Range Check for Target Selection:**
   - Implement a check to ensure that the selected target is within the reachable range of the robot. This prevents selecting targets that are beyond the robot's capabilities.

3. **Improve User Interaction:**
   - Modify the node to present an interface to the user immediately, eliminating the need to manually call the node for goal setting. This improvement enhances user experience and simplifies goal-setting procedures.

Incorporating these enhancements into your node would result in a more efficient and user-friendly navigation system for the robot.


