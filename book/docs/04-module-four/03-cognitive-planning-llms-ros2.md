--- 
id: 03-cognitive-planning-llms-ros2
title: "Cognitive Planning: Using LLMs to Translate Natural Language into ROS 2 Actions"
sidebar_label: "Chapter 3: Cognitive Planning"
sidebar_position: 3
---

# Cognitive Planning: Using LLMs to Translate Natural Language into ROS 2 Actions

## Introduction

Bridging the gap between high-level human instructions and low-level robot actions is a central challenge in robotics. **Cognitive planning** aims to solve this by enabling robots to understand abstract goals ("Clean the room"), reason about their environment, and generate a sequence of concrete, executable steps. Large Language Models (LLMs) are uniquely positioned to serve as the "brain" for such cognitive planning, translating natural language intent into structured robot behaviors, often expressed as ROS 2 actions.

## 3.1. Overview of Cognitive Planning with LLMs

Cognitive planning with LLMs involves leveraging the LLM's vast knowledge base and reasoning capabilities to:

1.  **Interpret high-level goals**: Understand the intent behind natural language commands.
2.  **Decompose tasks**: Break down complex goals into a series of simpler sub-tasks.
3.  **Generate action sequences**: Output a sequence of abstract or primitive robot actions.
4.  **Incorporate commonsense**: Use its training data's implicit commonsense knowledge to fill gaps and handle ambiguities.
5.  **Adapt to context**: Adjust plans based on the robot's current state and environment (though this often requires external context).

This process transforms human desires into robot instructions, making robots more autonomous and user-friendly.

## 3.2. Prompt Engineering for LLMs in Robotics

The effectiveness of using LLMs for cognitive planning heavily relies on **prompt engineering**. This involves crafting precise and informative prompts that guide the LLM to produce desired outputs. Key elements of effective prompts for robotics include:

-   **Role-playing**: Assigning the LLM a role (e.g., "You are a robot task planner").
-   **Clear Goal Definition**: Explicitly stating the robot's objective.
-   **Available Actions**: Providing a list or description of the robot's primitive actions, along with their parameters.
-   **Environmental Context**: Supplying relevant information about the robot's current state and environment.
-   **Output Format**: Specifying the desired format for the action sequence (e.g., a Python list of function calls, JSON).
-   **Few-shot examples**: Providing a few examples of input (natural language command) and desired output (robot action sequence) pairs.

### Example Prompt Structure

```
"You are a robot task planner. Your goal is to generate a sequence of robot actions to fulfill a user's request.
Available actions:
- navigate_to(location): Move the robot to a specified location (e.g., 'kitchen', 'bedroom').
- pick_up(object): Pick up a specified object.
- place_object(object, location): Place a picked-up object at a specified location.
- detect_object(object): Use computer vision to detect a specified object.
- report_status(message): Report a message to the user.

Current robot location: 'living_room'
Objects in living_room: 'remote_control', 'book'

User Request: 'Please find the remote control and bring it to me.'

Generate a step-by-step plan using only the available actions, outputting a Python list of function calls.
"
```

## 3.3. LLM-Generated Action Sequences

An LLM, given a well-engineered prompt, can generate an action sequence. This sequence might be abstract (e.g., `pick_up(remote_control)`) and needs to be further mapped to concrete ROS 2 actions.

```python
# Pseudo-code for LLM-generated abstract plan
llm_response = get_llm_response(prompt_for_cognitive_planning) # From Chapter 2's concept
print(f"LLM Generated Plan:\n{llm_response}\n")

# Example of an LLM-generated plan (string output from LLM)
# "[navigate_to('living_room'), detect_object('remote_control'), pick_up('remote_control'), navigate_to('user_location'), place_object('remote_control', 'user_hand')]"

def parse_llm_abstract_plan(llm_plan_string: str) -> list[tuple[str, dict]]:
    """
    Parses the LLM's string output into a list of (action_name, parameters) tuples.
    This would involve more robust parsing than simple string splitting in a real system.
    """
    abstract_actions = []
    # Simplified parsing for demonstration
    plan_elements = llm_plan_string.strip('[]').split('), ')
    for element in plan_elements:
        if '(' in element and ')' in element:
            action_name = element.split('(')[0].strip()
            params_str = element.split('(')[1].strip("')\"")
            params = {}
            # Very basic parameter parsing assuming key-value or single value
            if '=' in params_str:
                key, val = params_str.split('=')
                params[key.strip()] = val.strip()
            elif params_str:
                # Assuming single positional parameter like 'location' or 'object'
                params['target'] = params_str
            
            abstract_actions.append((action_name, params))
    return abstract_actions

# abstract_plan = parse_llm_abstract_plan(llm_response)
# print(f"Parsed Abstract Plan: {abstract_plan}")
```

## 3.4. Mapping Abstract Actions to ROS 2 Actions

The abstract actions generated by the LLM need to be mapped to the robot's actual capabilities, often exposed through ROS 2 interfaces (Actions, Services, Topics). This mapping layer translates the LLM's high-level intent into executable ROS 2 commands.

```mermaid
graph TD
    UserRequest[Natural Language Request] --> LLM[Large Language Model (Cognitive Planner)]
    LLM --> |Abstract Action Sequence| MappingLayer[Mapping Layer (Python Script)]
    MappingLayer --> |ROS 2 Actions/Goals| RobotController[Robot Control System (ROS 2)]
    RobotController --> Robot[Robot Hardware/Simulator]
```

### Example Mapping

-   `navigate_to(location)` could map to sending a `nav2_msgs/action/NavigateToPose` goal.
-   `pick_up(object)` could map to calling a `PickObject` ROS 2 service or action.
-   `detect_object(object)` could map to calling an `ObjectDetection` ROS 2 service or subscribing to an `ObjectDetection` topic.

## 3.5. Feedback Loops for Execution Monitoring

Robotic systems require feedback. The robot needs to report its status and progress, allowing the LLM-based planner to monitor execution and adapt if necessary. This might involve:

-   **Success/Failure Reporting**: Robot reports if an action succeeded or failed.
-   **Progress Updates**: Robot sends periodic updates (e.g., "Navigating to kitchen, 50% complete").
-   **Sensor Feedback**: LLM can request new sensor data to update its world model before planning the next step.

## Summary

Cognitive planning with LLMs enables robots to move beyond pre-programmed routines, interpreting natural language requests and generating complex action sequences autonomously. By effectively engineering prompts and creating robust mapping layers to ROS 2 actions, we can unlock advanced reasoning capabilities, bringing AI-robots closer to human-level intelligence and interaction. This integration culminates in the [Chapter 4: Capstone Project: The Autonomous Humanoid](04-capstone-autonomous-humanoid.md), where all these concepts are brought together in an end-to-end demonstration.

## Further Reading

-   [ROS 2 Actions Overview](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Actions.html)
-   [Google AI Blog: SayCan](https://ai.googleblog.com/2022/03/everyday-robots-saycan-what-can-i-do.html)
-   [Research Paper: Inner Monologue: Empowering LLMs as Active Reasoners for (Embodied) Agents](https://arxiv.org/abs/2210.05641)
-   [Prompt Engineering Guide](https://www.promptingguide.ai/)
