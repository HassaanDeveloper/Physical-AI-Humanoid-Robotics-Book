---
id: chapter-2-nodes-topics-services
title: Chapter 2 - ROS 2 Nodes, Topics, and Services
sidebar_label: ROS 2 Nodes, Topics, and Services
sidebar_position: 2
description: Understanding ROS 2 communication patterns for robotic systems
tags: [ros2, communication, nodes, topics, services, robotics]
keywords: [ROS 2, nodes, topics, services, communication, robotics]
learning_outcomes:
  - Understand the concept of ROS 2 nodes and their role in robotic systems
  - Implement publisher-subscriber communication using topics
  - Create client-server communication using services
  - Choose appropriate communication patterns for different use cases
prerequisites:
  - Basic understanding of ROS 2 concepts (Chapter 1)
  - Python programming knowledge
duration_minutes: 60
---

# Chapter 2 - ROS 2 Nodes, Topics, and Services

## Overview

In this chapter, we'll explore the core communication patterns in ROS 2: nodes, topics, and services. These patterns form the foundation of all ROS 2 applications and enable different components of a robotic system to communicate with each other effectively.

## Understanding ROS 2 Nodes

A ROS 2 node is an executable that uses ROS 2 client library to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system and typically perform specific tasks within the larger robotic system.

### Node Characteristics

- **Process**: Each node runs in its own process
- **Communication**: Nodes communicate with each other through topics, services, and other mechanisms
- **Organization**: Related functionality is grouped into nodes
- **Flexibility**: Nodes can be started, stopped, and restarted independently

### Creating a Node

Here's a basic structure of a ROS 2 node:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Node initialization code here
        self.get_logger().info('Node has been initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topic-Based Communication (Publisher-Subscriber Pattern)

Topics enable asynchronous, decoupled communication between nodes. Publishers send messages to topics, and subscribers receive messages from topics. This pattern is ideal for streaming data like sensor readings or robot state.

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher = self.create_publisher(String, 'sensor_data', 10)

        # Timer to publish data periodically
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Sensor reading at {self.get_clock().now().seconds_nanoseconds()}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    publisher = SensorPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DataProcessor(Node):
    def __init__(self):
        super().__init__('data_processor')
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received sensor data: {msg.data}')
        # Process the received data here

def main(args=None):
    rclpy.init(args=args)
    subscriber = DataProcessor()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service-Based Communication (Client-Server Pattern)

Services enable synchronous, request-response communication. A client sends a request to a server and waits for a response. This pattern is ideal for operations that require immediate responses or for actions that should be performed once.

### Service Server Example

First, define the service interface in a `.srv` file (e.g., `CalculateDistance.srv`):

```
float64 x1
float64 y1
float64 x2
float64 y2
---
float64 distance
```

Then implement the server:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger  # Using a built-in service for simplicity

class DistanceCalculatorService(Node):
    def __init__(self):
        super().__init__('distance_calculator_service')
        self.srv = self.create_service(
            Trigger,
            'calculate_distance',
            self.calculate_distance_callback
        )

    def calculate_distance_callback(self, request, response):
        # In a real implementation, this would calculate distance between coordinates
        self.get_logger().info('Calculating distance...')
        response.success = True
        response.message = 'Distance calculated successfully'
        return response

def main(args=None):
    rclpy.init(args=args)
    service = DistanceCalculatorService()

    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass
    finally:
        service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger

class DistanceRequester(Node):
    def __init__(self):
        super().__init__('distance_requester')
        self.cli = self.create_client(Trigger, 'calculate_distance')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = Trigger.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    requester = DistanceRequester()

    try:
        response = requester.send_request()
        if response is not None:
            print(f'Result: {response.success}, Message: {response.message}')
        else:
            print('Service call failed')
    except KeyboardInterrupt:
        pass
    finally:
        requester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Choosing Between Topics and Services

Understanding when to use each communication pattern is crucial:

### Use Topics (Publisher-Subscriber) when:
- Streaming continuous data (sensor readings, robot state)
- Multiple subscribers need the same information
- Real-time performance is important
- Decoupling publishers and subscribers is desired
- Data loss is acceptable (e.g., video frames)

### Use Services (Client-Server) when:
- Request-response pattern is needed
- Operation should be performed once
- Immediate response is required
- Error handling is important
- Operation has side effects that should only happen once

## Advanced Communication Patterns

### Actions
For long-running tasks with feedback, ROS 2 provides actions which combine features of topics and services:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
# Example with a hypothetical MoveRobot.action
# Actions have goal, feedback, and result components
```

### Parameters
For configuration values that can be changed at runtime:

```python
def __init__(self):
    super().__init__('param_example')
    self.declare_parameter('robot_speed', 1.0)
    self.speed = self.get_parameter('robot_speed').value
```

## Practical Exercise: Implement a Simple Navigation System

Let's combine what we've learned by implementing a simple navigation system:

1. **Sensor Node**: Publishes obstacle information
2. **Navigation Node**: Subscribes to sensor data and publishes movement commands
3. **Motor Controller Node**: Subscribes to movement commands and controls motors

This exercise demonstrates how nodes, topics, and services work together in a real robotic system.

## Learning Summary

In this chapter, you've learned:

1. Nodes are the fundamental building blocks of ROS 2 systems
2. Topics enable asynchronous publisher-subscriber communication
3. Services enable synchronous client-server communication
4. How to choose the appropriate communication pattern for different scenarios
5. Basic implementation of publishers, subscribers, services, and clients

## Exercises

1. Create a publisher that simulates temperature sensor readings
2. Create a subscriber that processes these readings and alerts if temperature is too high
3. Implement a service that calculates the average of the last 10 temperature readings
4. Build a simple robot simulator that moves based on velocity commands received via topics

## Next Steps

In the next chapter, we'll explore how to connect Python AI agents with ROS 2 using the rclpy library, bridging your AI knowledge with robotic systems.