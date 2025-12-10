---
id: 04-urdf-humanoids
title: "Understanding URDF (Unified Robot Description Format) for Humanoids"
sidebar_label: "Chapter 4: URDF for Humanoids"
sidebar_position: 4
---

# Understanding URDF (Unified Robot Description Format) for Humanoids

The Unified Robot Description Format (URDF) is an XML format used in ROS to describe all aspects of a robot. It's a critical tool for working with robots, especially humanoids, as it allows you to define their kinematic and dynamic properties, visual appearance, and collision characteristics. A well-defined URDF is essential for simulation, motion planning, and robot visualization.

## 4.1. What is URDF? Why is it Used?

URDF provides a standardized way to represent a robot's physical structure. This description is used by various ROS tools and libraries:

-   **Visualization**: Tools like RViz (ROS Visualization) use URDF to display a 3D model of your robot.
-   **Simulation**: Physics simulators (e.g., Gazebo) use URDF to understand the robot's mass, inertia, and joint properties for realistic simulation.
-   **Motion Planning**: Libraries like MoveIt! use the kinematic and dynamic information from URDF to plan collision-free trajectories for the robot's manipulators.
-   **Inverse Kinematics/Dynamics**: URDF defines the relationships between joints and links, which are fundamental for solving inverse kinematics (calculating joint angles for a desired end-effector pose) and dynamics problems.

For humanoids, URDF's ability to precisely define complex joint chains and body segments is invaluable, allowing for detailed modeling of legs, arms, and torso.

## 4.2. `link` and `joint` Elements: Definitions and Types

A URDF file is fundamentally composed of `<link>` and `<joint>` elements, which together describe the robot's kinematic chain.

### `link` Element

A `<link>` element describes a rigid body segment of the robot. This could be a robot's torso, a leg, an arm, a hand, or even a sensor. Each link has properties like:

-   **`inertial`**: Defines the mass, center of mass, and inertia matrix of the link. Crucial for physics simulation.
    ```xml
    <inertial>
        <mass value="1.0"/>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        <inertia ixx="0.003" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.003"/>
    </inertial>
    ```
-   **`visual`**: Describes the 3D model (e.g., a `.dae` or `.stl` file) and color properties for visualization in tools like RViz.
    ```xml
    <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
            <mesh filename="package://my_robot_description/meshes/base_link.stl"/>
        </geometry>
        <material name="blue">
            <color rgba="0 0 0.8 1"/>
        </material>
    </visual>
    ```
-   **`collision`**: Defines the 3D model used for collision detection. Often a simplified version of the visual geometry to reduce computation.
    ```xml
    <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
            <mesh filename="package://my_robot_description/meshes/base_link_collision.stl"/>
        </geometry>
    </collision>
    ```

### `joint` Element

A `<joint>` element connects two links: a `parent` link and a `child` link. It defines the axis of rotation or translation between them, as well as limits and dynamics.

-   **`name`**: Unique identifier for the joint.
-   **`type`**: The type of joint. Common types include:
    -   **`revolute`**: A rotating joint with a single axis of rotation and a limited range of motion. (e.g., elbow, knee).
    -   **`continuous`**: A rotating joint with a single axis of rotation and an unlimited range of motion (e.g., a wheel).
    -   **`prismatic`**: A sliding joint that moves along a single axis (e.g., a linear actuator).
    -   **`fixed`**: A joint that rigidly connects two links. No relative motion is allowed. This is crucial for connecting base links to the world, or multiple parts of a rigid structure.
    -   **`planar`**: Allows motion in a plane.
    -   **`floating`**: Allows all 6 degrees of freedom.
-   **`origin`**: Specifies the transform from the parent link's origin to the child link's origin.
-   **`axis`**: Defines the axis of rotation or translation for revolute, continuous, and prismatic joints.
-   **`limit`**: For revolute and prismatic joints, defines the upper and lower bounds of motion, velocity, and effort.
-   **`dynamics`**: (Optional) Friction and damping parameters for physics simulation.

```xml
<joint name="torso_to_head_joint" type="revolute">
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <parent link="torso_link"/>
    <child link="head_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" velocity="1.0" effort="10.0"/>
</joint>
```

## 4.3. Practical Example: Building a Simple Humanoid URDF

Let's consider a very simple humanoid robot with a `base_link` (torso), a `head_link`, and two `arm_link`s.

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

    <!-- Base Link (Torso) -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.2 0.1 0.4"/>
            </geometry>
            <material name="gray">
                <color rgba="0.5 0.5 0.5 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.2 0.1 0.4"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="10.0"/>
            <origin xyz="0 0 0"/>
            <inertia ixx="1.0" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="1.0"/>
        </inertial>
    </link>

    <!-- Head Link -->
    <link name="head_link">
        <visual>
            <geometry>
                <sphere radius="0.1"/>
            </geometry>
            <material name="red">
                <color rgba="0.8 0 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <sphere radius="0.1"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="1.0"/>
            <origin xyz="0 0 0"/>
            <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
        </inertial>
    </link>

    <!-- Joint: Base to Head -->
    <joint name="base_to_head_joint" type="revolute">
        <origin xyz="0 0 0.25" rpy="0 0 0"/> <!-- Position relative to parent link's origin -->
        <parent link="base_link"/>
        <child link="head_link"/>
        <axis xyz="0 0 1"/> <!-- Yaw rotation -->
        <limit lower="-1.57" upper="1.57" velocity="0.5" effort="100"/>
    </joint>

    <!-- Right Arm Link -->
    <link name="right_arm_link">
        <visual>
            <geometry>
                <cylinder radius="0.03" length="0.3"/>
            </geometry>
            <material name="green">
                <color rgba="0 0.8 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.03" length="0.3"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="0.5"/>
            <origin xyz="0 0 0.15"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
    </link>

    <!-- Joint: Base to Right Arm -->
    <joint name="base_to_right_arm_joint" type="revolute">
        <origin xyz="-0.12 0 0.15" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="right_arm_link"/>
        <axis xyz="0 1 0"/> <!-- Pitch rotation -->
        <limit lower="-1.57" upper="1.57" velocity="0.5" effort="100"/>
    </joint>

    <!-- Left Arm Link (similar to right arm) -->
    <link name="left_arm_link">
        <visual>
            <geometry>
                <cylinder radius="0.03" length="0.3"/>
            </geometry>
            <material name="green">
                <color rgba="0 0.8 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.03" length="0.3"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="0.5"/>
            <origin xyz="0 0 0.15"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
    </link>

    <!-- Joint: Base to Left Arm -->
    <joint name="base_to_left_arm_joint" type="revolute">
        <origin xyz="0.12 0 0.15" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="left_arm_link"/>
        <axis xyz="0 1 0"/> <!-- Pitch rotation -->
        <limit lower="-1.57" upper="1.57" velocity="0.5" effort="100"/>
    </joint>

</robot>
```

### Loading and Visualizing URDF in ROS 2

To visualize this simple humanoid robot in ROS 2:

1.  **Save the URDF**: Save the XML code above as `simple_humanoid.urdf` in `code/module1/chapter4/`.
2.  **Launch `robot_state_publisher` and `joint_state_publisher_gui`**: These ROS 2 packages are essential for visualizing URDFs.
    ```bash
    # In a terminal (after sourcing ROS 2 environment)
    ros2 launch urdf_tutorial display.launch.py model:=$(ros2 pkg prefix your_package_name)/share/your_package_name/simple_humanoid.urdf
    # Note: Replace 'your_package_name' and path with the actual location of your URDF
    # A simpler way if you have a local URDF:
    # ros2 launch urdf_tutorial display.launch.py model:=/path/to/your/simple_humanoid.urdf
    ```
    Alternatively, you can manually run the publishers and RViz:
    ```bash
    # Terminal 1: Publish joint states (GUI allows you to move joints)
    ros2 run joint_state_publisher_gui joint_state_publisher_gui

    # Terminal 2: Publish robot state (takes joint states and URDF to publish transforms)
    ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="`cat simple_humanoid.urdf`"

    # Terminal 3: Launch RViz to visualize the robot
    rviz2
    # In RViz, add a "RobotModel" display and ensure its "Description Topic" is set to "robot_description"
    # and "Fixed Frame" is set to "base_link" (or the first link in your URDF)
    ```

## Summary

URDF is a powerful and flexible XML format for describing robots in ROS 2. By defining links (rigid bodies) and joints (connections between links) with their inertial, visual, and collision properties, you can create detailed robot models for simulation, visualization, and control. This foundational understanding of URDF is crucial for developing and working with complex robotic systems, especially humanoids, in AI-native robotics.

## Further Reading

-   [Introduction to ROS 2 Middleware for Robot Control](01-ros2-middleware-intro.md)
-   [ROS 2 Nodes, Topics, and Services Deep-Dive](02-nodes-topics-services.md)
-   [Bridging Python Agents to ROS Controllers using rclpy](03-python-rclpy-integration.md)
-   [ROS 2 URDF Tutorial](https://docs.ros.org/en/humble/Tutorials/URDF/URDF-C++-tutorial.html) (C++ based, but concepts apply)
-   [URDF XML Specification](http://wiki.ros.org/urdf/XML)
-   [Using URDF with MoveIt!](https://moveit.ros.org/documentation/getting_started/urdf/)
