---
id: 05-module-three-glossary
title: "Module 3 Glossary of Terms"
sidebar_label: "Glossary"
sidebar_position: 5
---

# Module 3: The AI-Robot Brain (NVIDIA Isaac™) - Glossary

This glossary provides definitions for key terms and concepts introduced in Module 3.

## A

**Action (ROS 2)**: A high-level, asynchronous communication mechanism in ROS 2 for long-running tasks, allowing clients to send goals, receive feedback, and eventually get results.

**Advanced Perception**: Techniques that go beyond basic sensor readings to create a rich, actionable understanding of the environment, often involving sensor fusion, semantic segmentation, and 3D reconstruction.

## B

**Behavioral Cloning (BC)**: An imitation learning technique where an agent learns to mimic expert demonstrations by directly mapping observations to actions, typically using supervised learning.

**Bipedal Humanoid Movement**: Complex locomotion capabilities of two-legged robots, involving challenges in maintaining balance, gait generation, and coordinating many degrees of freedom.

**Bundle Adjustment**: An optimization technique used in VSLAM to refine the estimated camera poses and 3D map points by minimizing the reprojection error of observed features.

## C

**Costmaps (Nav2)**: Environmental representations used by Nav2 to indicate areas that are safe, occupied by obstacles, or uncertain, guiding a robot's path planning.

## D

**Deep Learning for Perception**: Application of deep neural networks to extract high-level features and understanding from raw sensor data (e.g., image, LiDAR point clouds) for tasks like object detection, segmentation, and classification.

**Direct Methods (VSLAM)**: VSLAM techniques that use pixel intensity values directly (without explicit feature extraction) to estimate camera motion and map structure.

**Domain Randomization**: A technique used in simulation (e.g., Isaac Sim) to randomize various aspects of the environment (lighting, textures, object properties) during data collection to improve the generalization of AI models to the real world.

## F

**Feature-based Methods (VSLAM)**: VSLAM techniques that detect and track distinct visual features (e.g., corners, edges) in camera images to estimate camera motion and build a map.

## I

**Imitation Learning (IL)**: A training methodology where an agent learns a task by observing and mimicking expert demonstrations, often used when defining a reward function for RL is difficult.

**Inverse Reinforcement Learning (IRL)**: A technique used in imitation learning to infer the underlying reward function that explains observed expert behavior.

**Isaac ROS**: A collection of hardware-accelerated ROS 2 packages developed by NVIDIA, optimizing critical robotics tasks like perception, navigation, and manipulation using NVIDIA GPUs.

**Isaac Sim (NVIDIA Isaac Sim)**: A scalable robotics simulation application and synthetic data generation tool built on NVIDIA Omniverse, providing photorealistic environments for AI training.

## L

**LiDAR (Light Detection and Ranging)**: A remote sensing technology that uses pulsed laser light to measure distances, creating detailed 3D point clouds of the environment, crucial for mapping and navigation.

**Loop Closure (VSLAM)**: The process in VSLAM of detecting when a robot returns to a previously visited location, used to correct accumulated error in the map and trajectory, ensuring global consistency.

## N

**Nav2**: The ROS 2 navigation stack, providing a modular framework for autonomous mobile robot navigation, including global and local path planning, obstacle avoidance, and control.

**NVIDIA Omniverse**: A platform for connecting and building 3D tools and applications, enabling virtual collaboration and physically accurate simulation, upon which Isaac Sim is built.

## O

**Object Detection and Tracking**: Identifying and localizing specific objects within an environment (detection) and continuously monitoring their position and motion over time (tracking).

## P

**Path Planning**: The process of finding a collision-free path for a robot from a start location to a goal location, often involving global and local planners.

## R

**Reinforcement Learning (RL)**: A machine learning paradigm where an agent learns to make decisions by performing actions in an environment to maximize a cumulative reward signal through trial and error.

## S

**Semantic Segmentation**: A computer vision technique that classifies each pixel in an image or point in a point cloud with a specific category label (e.g., "road," "person," "sky"), providing a high-level understanding of the scene.

**Sensor Fusion**: Combining data from multiple disparate sensors to achieve a more accurate, robust, and complete understanding of an environment or system state than would be possible with individual sensors.

**Sim-to-Real Transfer**: The process of training an AI model or robot control policy in a simulated environment and then effectively deploying it on a real physical robot.

**Synthetic Data Generation (SDG)**: The creation of artificial data in simulation, often with ground truth labels, used to train AI models when real-world data collection is expensive, dangerous, or impractical.

## T

**Transfer Learning**: A machine learning technique where a model trained on one task is reused as the starting point for a model on a second, related task, often to accelerate training or improve performance with limited data.

## V

**VSLAM (Visual Simultaneous Localization and Mapping)**: A technique that allows a robot to concurrently estimate its own position and orientation within an environment while simultaneously building a map of that environment, using visual sensor data from cameras.
