---
id: 03-python-rclpy-integration
title: "Bridging Python Agents to ROS Controllers using rclpy"
sidebar_label: "Chapter 3: Python rclpy"
sidebar_position: 3
---

# Bridging Python Agents to ROS Controllers using rclpy

In the realm of AI-native robotics, Python is often the language of choice for developing intelligent agents due to its rich ecosystem of AI/ML libraries. To enable these Python agents to interact with ROS 2-powered robots, we use `rclpy`, the official Python client library for ROS 2. `rclpy` allows Python programs to create ROS 2 nodes, publish and subscribe to topics, call and provide services, and interact with parameters and actions. This enables seamless control of robotic systems, including complex humanoid robots, whose physical structure is often defined using URDF, as we will explore in [Chapter 4: Understanding URDF (Unified Robot Description Format) for Humanoids](04-urdf-humanoids.md).

## 3.1. `rclpy` Overview and Setup

`rclpy` is designed to be idiomatic Python, providing a straightforward API for ROS 2 communication. It builds upon `rcl` (ROS Client Library), which is a C API that provides the common functionality for all client libraries.

### Setup

Before you can use `rclpy`, you need to have ROS 2 installed and sourced in your environment. Assuming you have ROS 2 (e.g., Humble Hawksbill) installed, you typically don't need to install `rclpy` separately as it comes with the ROS 2 Python development tools.

To use `rclpy` within your Python projects, ensure your ROS 2 environment is sourced:

```bash
# For Linux/macOS
source /opt/ros/humble/setup.bash

# For Windows (PowerShell)
# .\install\setup.ps1
```

You can then import `rclpy` in your Python scripts:

```python
import rclpy
from rclpy.node import Node
# Other imports like message types (e.g., from std_msgs.msg import String)
```

## 3.2. Creating a Simple Publisher Node in Python

A publisher node sends messages to a topic. Here's how to create a basic ROS 2 publisher in Python using `rclpy`. This node will publish a simple "Hello ROS 2" string to the `/topic` topic every 0.5 seconds.

**`minimal_publisher.py`**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Import the String message type

class MinimalPublisher(Node):
    def __init__(self):
        # Initialize the node with the name 'minimal_publisher'
        super().__init__('minimal_publisher')
        # Create a publisher for the 'topic' topic, using String messages, with a queue size of 10
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        # Create a timer that calls the timer_callback method every 0.5 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0 # Counter for messages

    def timer_callback(self):
        # Create a new String message
        msg = String()
        msg.data = 'Hello ROS 2: %d' % self.i # Set the message data
        self.publisher_.publish(msg) # Publish the message
        self.get_logger().info('Publishing: "%s"' % msg.data) # Log the published message
        self.i += 1 # Increment counter

def main(args=None):
    rclpy.init(args=args) # Initialize rclpy
    minimal_publisher = MinimalPublisher() # Create the publisher node
    rclpy.spin(minimal_publisher) # Keep the node alive until Ctrl+C is pressed
    # Destroy the node and shutdown rclpy when done
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To run this node, save it as `minimal_publisher.py` and then execute:

```bash
python3 minimal_publisher.py
```

## 3.3. Creating a Simple Subscriber Node in Python

A subscriber node receives messages from a topic. This node will listen to the `/topic` topic and print any incoming string messages.

**`minimal_subscriber.py`**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Import the String message type

class MinimalSubscriber(Node):
    def __init__(self):
        # Initialize the node with the name 'minimal_subscriber'
        super().__init__('minimal_subscriber')
        # Create a subscription to the 'topic' topic, using String messages
        # The 'listener_callback' method will be called when a message is received
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10) # Queue size of 10
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Log the received message
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args) # Initialize rclpy
    minimal_subscriber = MinimalSubscriber() # Create the subscriber node
    rclpy.spin(minimal_subscriber) # Keep the node alive until Ctrl+C is pressed
    # Destroy the node and shutdown rclpy when done
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To run this node, save it as `minimal_subscriber.py` and then execute:

```bash
python3 minimal_subscriber.py
```

Run the publisher and subscriber simultaneously in two different terminals to observe the communication.

## 3.4. Implementing a Basic Service Client and Server in Python

Services allow for synchronous request-response communication. Here, we'll demonstrate a simple service server that adds two integers and a client that requests this operation.

**`minimal_service.py` (Service Server)**:
```python
from example_interfaces.srv import AddTwoInts # Import the service type
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        # Initialize the node with the name 'minimal_service'
        super().__init__('minimal_service')
        # Create a service named 'add_two_ints' using the AddTwoInts service type
        # The 'add_two_ints_callback' method will handle incoming requests
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service server started.')

    def add_two_ints_callback(self, request, response):
        # Perform the addition and set the response sum
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request: a: %d b: %d' % (request.a, request.b))
        return response # Return the response

def main(args=None):
    rclpy.init(args=args) # Initialize rclpy
    minimal_service = MinimalService() # Create the service node
    rclpy.spin(minimal_service) # Keep the node alive
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**`minimal_client.py` (Service Client)**:
```python
import sys
from example_interfaces.srv import AddTwoInts # Import the service type
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        # Initialize the node with the name 'minimal_client_async'
        super().__init__('minimal_client_async')
        # Create a client for the 'add_two_ints' service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request() # Create a request object

    def send_request(self):
        # Set the request arguments from command line
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        # Call the service asynchronously
        self.future = self.cli.call_async(self.req)
        # Wait until the future is complete (response received)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result() # Return the result

def main(args=None):
    rclpy.init(args=args) # Initialize rclpy
    minimal_client = MinimalClientAsync() # Create the client node
    response = minimal_client.send_request() # Send request and get response
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (minimal_client.req.a, minimal_client.req.b, response.sum)) # Log the result
    minimal_client.destroy_node() # Destroy the node
    rclpy.shutdown() # Shutdown rclpy

if __name__ == '__main__':
    main()
```

To run these:
1.  In one terminal, start the service server: `python3 minimal_service.py`
2.  In another terminal, call the client with arguments: `python3 minimal_client.py 5 7` (this will add 5 and 7).

## 3.5. Error Handling and Best Practices in `rclpy`

-   **Node Initialization and Shutdown**: Always ensure `rclpy.init()` and `rclpy.shutdown()` are called. Nodes should be destroyed using `node.destroy_node()`.
-   **Logging**: Use `self.get_logger().info()`, `warn()`, `error()`, etc., for proper ROS 2 logging.
-   **Parameter Handling**: Utilize ROS 2 parameters for configurable node behaviors rather than hardcoding values.
-   **Message Definition**: Prefer existing ROS 2 message types when possible. Define custom messages only when necessary.
-   **QoS Profiles**: Carefully select QoS profiles for publishers and subscribers based on your application's requirements for reliability, latency, and throughput.
-   **Asynchronous Operations**: For long-running tasks, consider using `Actions` instead of `Services` to provide periodic feedback and allow for cancellation.

## Summary

`rclpy` provides a powerful and intuitive Python interface for interacting with ROS 2. By mastering the creation of nodes, publishers, subscribers, and service clients/servers, you can effectively bridge your Python AI agents with the underlying robot control infrastructure, enabling sophisticated autonomous behaviors. Adhering to best practices in error handling and resource management will ensure the robustness of your robot applications.

## Further Reading

-   [rclpy documentation](https://docs.ros.org/en/humble/p/rclpy/index.html)
-   [ROS 2 Tutorials: Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Writing-A-Simple-Publisher-And-Subscriber--Python.html)
-   [ROS 2 Tutorials: Writing a simple service and client (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Writing-A-Simple-Service-And-Client--Python.html)
