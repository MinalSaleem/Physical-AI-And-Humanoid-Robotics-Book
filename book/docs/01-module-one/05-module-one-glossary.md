---
id: 05-module-one-glossary
title: "Module 1 Glossary of Terms"
sidebar_label: "Glossary"
sidebar_position: 5
---

# Module 1: The Robotic Nervous System (ROS 2) - Glossary

This glossary provides definitions for key terms and concepts introduced in Module 1.

## A

**Action**: A high-level, asynchronous communication mechanism in ROS 2 designed for long-running tasks. It involves a client sending a goal, receiving continuous feedback, and eventually a result.

## D

**DDS (Data Distribution Service)**: The default underlying middleware used by ROS 2 for its transport layer, enabling efficient and reliable communication between nodes.

## L

**Link**: A rigid body segment of a robot as defined in URDF. It has properties like mass, inertia, visual representation, and collision geometry.

## N

**Node**: An individual executable program in ROS 2 that performs a specific task. Nodes are the fundamental building blocks of a ROS 2 application.

## P

**Publisher**: A ROS 2 entity that sends messages to a named topic, enabling one-to-many, asynchronous data streaming.

## Q

**Quality of Service (QoS) Policies**: Settings in ROS 2 that allow configuration of message exchange behavior over topics, controlling aspects like reliability, latency, and data persistence.

## R

**rclpy**: The official Python client library for ROS 2, allowing Python programs to create and interact with ROS 2 nodes, topics, and services.

**ROS 2 (Robot Operating System 2)**: A flexible framework for writing robot software, providing a set of libraries, tools, and conventions to create complex and robust robot applications.

## S

**Service**: A synchronous, request/response communication mechanism in ROS 2 where a client sends a request to a server node, and the server processes it and sends back a response.

**Subscriber**: A ROS 2 entity that listens to a named topic to receive messages sent by publishers.

## T

**Topic**: A named data bus in ROS 2 used for asynchronous, one-to-many communication between nodes, carrying messages (data packets).

## U

**URDF (Unified Robot Description Format)**: An XML file format used in ROS to describe the kinematic and dynamic properties, visual appearance, and collision characteristics of a robot.
