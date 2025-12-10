# ROS 2 Action Definitions Placeholder

This directory is a placeholder for custom ROS 2 Action definitions (e.g., `.action` files)
that would be required for the cognitive planning examples in Chapter 3.

**Example of a simple ROS 2 action definition (`PickUpObject.action`):**

```
# Goal
string object_name
---
# Result
bool success
string message
---
# Feedback
float32 progress
string status_message
```

In a real ROS 2 project, these `.action` files would be defined in a ROS 2 package and built,
allowing Python nodes to create action clients and servers. This allows for communication
between the LLM-based planner and the robot's low-level execution modules.