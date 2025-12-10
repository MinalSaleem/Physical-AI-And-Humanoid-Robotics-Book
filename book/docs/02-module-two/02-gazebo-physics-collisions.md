---
id: 02-gazebo-physics-collisions
title: "Simulating Physics, Gravity, and Collisions in Gazebo"
sidebar_label: "Chapter 2: Gazebo Physics"
sidebar_position: 2
---

# Simulating Physics, Gravity, and Collisions in Gazebo

Realistic physics simulation is fundamental to the digital twin concept. It allows engineers and researchers to test robot designs, control algorithms, and interaction with the environment in a virtual space that closely mimics the real world. Gazebo excels in this area, offering robust physics engines and extensive configuration options. For more advanced visualization and human-robot interaction, see [Chapter 3: High-Fidelity Rendering and Human-Robot Interaction in Unity](03-unity-rendering-hri.md). To integrate and simulate various sensors in these digital twin environments, refer to [Chapter 4: Simulating Sensors: LiDAR, Depth Cameras, and IMUs](04-simulating-sensors.md).

## 2.1. Gazebo Physics Engines

Gazebo doesn't implement its own physics engine from scratch. Instead, it acts as a wrapper around several high-performance, open-source physics libraries. You can choose the engine that best suits your simulation needs, based on factors like stability, speed, and feature set. Common options include:

-   **ODE (Open Dynamics Engine)**: The default engine in Gazebo, known for its speed and ability to handle rigid body dynamics with contacts.
-   **Bullet**: A popular open-source physics engine used in many games and simulations. Offers good performance and features.
-   **Simbody**: Designed for high-performance simulation of biological and biomechanical systems.
-   **DART (Dynamic Animation and Robotics Toolkit)**: Optimized for robotics and biomechanics research, providing advanced capabilities for contact and collision handling.

The choice of physics engine is typically specified in your `.world` file within the `<physics>` tag.

```xml
<physics name="default_physics" default="true" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
      <sor>1.3</sor>
      <min_depth>0.001</min_depth>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

-   **`max_step_size`**: The maximum time step allowed for the physics simulation. Smaller values lead to more accurate but slower simulations.
-   **`real_time_factor`**: The ratio of simulated time to real time. `1.0` means the simulation runs at real-time speed. Values greater than `1.0` mean faster-than-real-time.
-   **`real_time_update_rate`**: The frequency at which the physics engine is updated.

## 2.2. Setting Gravity

Gravity is a fundamental force in any realistic physics simulation. In Gazebo, gravity is defined in the `<world>` tag of your `.world` file.

```xml
<gravity>0 0 -9.8</gravity> <!-- x y z components of gravity vector -->
```
The default value `-9.8` in the z-direction simulates Earth's gravity. You can change these values to simulate different gravitational environments or even zero gravity.

## 2.3. Defining Link Mass and Inertia

For objects to interact realistically with physics, they must have defined mass and inertial properties. These are specified within the `<inertial>` tag of a `<link>` in your robot's SDF/URDF model.

-   **`mass`**: The total mass of the link in kilograms.
-   **`inertia`**: A 3x3 inertia tensor matrix (represented by `ixx`, `iyy`, `izz`, `ixy`, `ixz`, `iyz`) that describes how difficult it is to change the rotational motion of the link. It's crucial for realistic rotational dynamics. For simple geometric shapes, you can often find formulas to calculate these values.

Example from Chapter 1's `cube_world.world` model:
```xml
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.00666</ixx> <iyy>0.00666</iyy> <izz>0.00666</izz>
            <ixy>0.0</ixy> <ixz>0.0</ixz> <iyz>0.0</iyz>
          </inertia>
        </inertial>
```

## 2.4. Collision Geometries and Contact Sensors

Collision detection is the process by which a physics engine determines if two objects are in contact or interpenetrating. In Gazebo, collision properties are defined in the `<collision>` tag within a `<link>`.

-   **`geometry`**: Defines the shape of the collision body (e.g., `box`, `sphere`, `cylinder`, `mesh`). It's often a simplified version of the `<visual>` geometry to reduce computational load.
-   **`surface`**: (Optional) Specifies physical properties of the surface like friction, restitution (bounciness), and contact parameters.

```xml
<collision name="my_collision_body">
    <geometry>
        <box>
            <size>0.2 0.2 0.2</size>
        </box>
    </geometry>
    <surface>
        <friction>
            <ode>
                <mu>0.9</mu> <!-- Coefficient of friction -->
                <mu2>0.9</mu2>
            </ode>
        </friction>
        <bounce>
            <restitution_coefficient>0.1</restitution_coefficient> <!-- Bounciness -->
            <threshold>0.05</threshold>
        </bounce>
    </surface>
</collision>
```

### Contact Sensors

To detect collisions programmatically within your robot's software, you can attach a **contact sensor** to a link. A contact sensor will report when specific parts of your robot (or other objects in the world) are touching.

Contact sensor definition in SDF:
```xml
<sensor name="my_contact_sensor" type="contact">
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <contact>
    <collision>my_link::my_collision_body</collision>
  </contact>
  <plugin name="gazebo_ros_bumper" filename="libgazebo_ros_bumper.so">
    <ros>
      <namespace>/my_robot</namespace>
      <argument>~/out:=contact_sensor_data</argument>
    </ros>
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <bumperTopicName>contact_sensor_data</bumperTopicName>
  </plugin>
</sensor>
```
This sensor will publish contact information to the `/my_robot/contact_sensor_data` ROS 2 topic if `libgazebo_ros_bumper.so` is used.

## Summary

Accurate physics simulation, including gravity and collision handling, is critical for developing robust robotic systems. Gazebo provides powerful tools to define and configure these aspects through its choice of physics engines, `.world` file configurations for gravity, and detailed `<inertial>` and `<collision>` properties within robot models. Understanding these elements allows you to create highly realistic virtual testbeds for your AI-native robots.

## Further Reading

-   [Gazebo Physics Tutorial](http://gazebosim.org/tutorials?tut=physics_params&cat=physics)
-   [SDF Specification: Link](http://sdformat.org/spec?ver=1.7&elem=link)
-   [SDF Specification: Sensor](http://sdformat.org/spec?ver=1.7&elem=sensor)
-   [ROS 2 Gazebo Plugins](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki)
