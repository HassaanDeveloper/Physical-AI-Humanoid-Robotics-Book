---
id: chapter-3-python-agents-rclpy
title: Chapter 3 - Python Agents with rclpy
sidebar_label: Python Agents with rclpy
sidebar_position: 3
description: Bridging Python AI agents with ROS 2 using rclpy
tags: [ros2, python, rclpy, ai, agents, robotics]
keywords: [ROS 2, Python, rclpy, AI agents, robotics integration]
learning_outcomes:
  - Understand how to use rclpy to connect Python AI agents to ROS 2
  - Implement Python-based AI agents that communicate with ROS 2 nodes
  - Receive sensor data from ROS 2 in Python AI agents
  - Send control commands from Python AI agents to ROS 2
prerequisites:
  - Understanding of ROS 2 communication patterns (Chapter 2)
  - Proficiency in Python programming
  - Basic knowledge of AI/ML concepts
duration_minutes: 75
---

# Chapter 3 - Python Agents with rclpy

## Overview

In this chapter, we'll explore how to bridge your existing Python AI knowledge with ROS 2 using the rclpy library. This integration allows you to leverage powerful Python AI libraries while benefiting from ROS 2's robotics infrastructure. We'll learn how to create Python-based AI agents that can receive sensor data from robots and send control commands to robotic systems.

## Introduction to rclpy

rclpy is the Python client library for ROS 2. It provides Python bindings for the ROS 2 middleware and enables Python programs to interact with the ROS 2 ecosystem. With rclpy, you can:

- Create ROS 2 nodes in Python
- Publish and subscribe to topics
- Provide and call services
- Work with actions
- Handle parameters

### Installation and Setup

rclpy is typically installed as part of a ROS 2 distribution. To use it in your Python projects:

```bash
# Make sure ROS 2 is sourced
source /opt/ros/humble/setup.bash  # Replace 'humble' with your ROS 2 distribution
```

## Creating AI Agents with rclpy

An AI agent in the ROS 2 context is a node that implements decision-making logic. The agent receives observations from sensors, processes them using AI algorithms, and sends actions to actuators.

### Basic AI Agent Structure

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import numpy as np

class SimpleAIAgent(Node):
    def __init__(self):
        super().__init__('simple_ai_agent')

        # Subscribers for sensor data
        self.sensor_subscription = self.create_subscription(
            Float32,
            'sensor_input',
            self.sensor_callback,
            10
        )

        # Publishers for control commands
        self.command_publisher = self.create_publisher(
            Float32,
            'motor_command',
            10
        )

        # Internal state
        self.current_sensor_value = 0.0

        # Timer for AI processing loop
        self.ai_timer = self.create_timer(0.1, self.ai_processing_loop)

    def sensor_callback(self, msg):
        """Process incoming sensor data"""
        self.current_sensor_value = msg.data
        self.get_logger().info(f'Received sensor value: {self.current_sensor_value}')

    def ai_processing_loop(self):
        """Main AI decision-making loop"""
        # Apply AI logic to determine action
        action = self.decide_action(self.current_sensor_value)

        # Publish the resulting command
        cmd_msg = Float32()
        cmd_msg.data = action
        self.command_publisher.publish(cmd_msg)
        self.get_logger().info(f'Published command: {action}')

    def decide_action(self, sensor_value):
        """Simple AI decision function"""
        # Placeholder AI logic - in practice, this could be a neural network,
        # reinforcement learning algorithm, or other AI method
        if sensor_value > 0.5:
            return -1.0  # Move backward
        elif sensor_value < -0.5:
            return 1.0   # Move forward
        else:
            return 0.0   # Stop

def main(args=None):
    rclpy.init(args=args)
    ai_agent = SimpleAIAgent()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integrating Popular Python AI Libraries

One of the key advantages of using Python with ROS 2 is the ability to leverage popular AI libraries:

### TensorFlow/Keras Integration

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import tensorflow as tf
import numpy as np
from cv_bridge import CvBridge
import cv2

class VisionAIAgent(Node):
    def __init__(self):
        super().__init__('vision_ai_agent')

        # Load pre-trained model
        self.model = tf.keras.models.load_model('path/to/model.h5')
        self.bridge = CvBridge()

        # Subscriptions and publications
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def image_callback(self, msg):
        """Process camera image and make navigation decision"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Preprocess image for the model
            resized_image = cv2.resize(cv_image, (224, 224))
            normalized_image = resized_image.astype(np.float32) / 255.0
            input_tensor = np.expand_dims(normalized_image, axis=0)

            # Run inference
            prediction = self.model.predict(input_tensor)

            # Interpret prediction and generate command
            cmd_vel = Twist()
            if prediction[0][0] > 0.7:  # If obstacle detected
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.5  # Turn right
            else:
                cmd_vel.linear.x = 0.5  # Move forward
                cmd_vel.angular.z = 0.0

            # Publish command
            self.cmd_vel_pub.publish(cmd_vel)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
```

### PyTorch Integration

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import torch
import numpy as np

class RLAgent(Node):
    def __init__(self):
        super().__init__('rl_agent')

        # Load PyTorch model
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.policy_network = torch.load('path/to/policy.pth', map_location=self.device)
        self.policy_network.eval()

        # ROS interfaces
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.latest_scan = None

    def scan_callback(self, msg):
        """Process laser scan and determine action using neural network"""
        # Convert scan to tensor
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=np.inf)  # Replace NaN with inf
        ranges = np.clip(ranges, 0, 10.0)  # Clip to reasonable range

        # Normalize and convert to tensor
        normalized_ranges = ranges / 10.0
        input_tensor = torch.tensor(normalized_ranges, dtype=torch.float32).unsqueeze(0).to(self.device)

        # Get action from policy network
        with torch.no_grad():
            action = self.policy_network(input_tensor)
            linear_vel, angular_vel = action.squeeze().cpu().numpy()

        # Create and publish twist message
        cmd_vel = Twist()
        cmd_vel.linear.x = float(linear_vel)
        cmd_vel.angular.z = float(angular_vel)
        self.cmd_pub.publish(cmd_vel)
```

## Handling Different Message Types

ROS 2 supports various message types for different sensor modalities:

### Working with PointCloud2 Messages

```python
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def pointcloud_callback(self, msg):
    """Process 3D point cloud data"""
    # Convert PointCloud2 to list of points
    points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

    # Process points with AI algorithm
    obstacles = self.detect_obstacles(points)

    # Generate appropriate response
    self.navigate_around_obstacles(obstacles)
```

### Working with Joint State Messages

```python
from sensor_msgs.msg import JointState

def joint_state_callback(self, msg):
    """Process robot joint states"""
    joint_positions = dict(zip(msg.name, msg.position))

    # Use AI to determine next action based on joint states
    target_positions = self.compute_target_joints(joint_positions)

    # Publish joint commands
    self.publish_joint_commands(target_positions)
```

## Advanced AI Agent Patterns

### Stateful Agents with Memory

```python
class StatefulAIAgent(Node):
    def __init__(self):
        super().__init__('stateful_ai_agent')

        # Maintain state over time
        self.state_history = []
        self.max_history = 100

        # Subscribe to sensor data
        self.sub = self.create_subscription(Float32, 'sensor', self.callback, 10)
        self.pub = self.create_publisher(Float32, 'command', 10)

    def callback(self, msg):
        # Add current observation to history
        self.state_history.append({
            'timestamp': self.get_clock().now(),
            'value': msg.data
        })

        # Limit history size
        if len(self.state_history) > self.max_history:
            self.state_history.pop(0)

        # Make decision based on historical data
        action = self.decide_based_on_history()

        # Publish action
        cmd_msg = Float32()
        cmd_msg.data = action
        self.pub.publish(cmd_msg)

    def decide_based_on_history(self):
        """Make decision based on historical data"""
        if len(self.state_history) < 10:
            return 0.0  # Not enough data yet

        # Example: Moving average of recent values
        recent_values = [item['value'] for item in self.state_history[-10:]]
        avg_value = sum(recent_values) / len(recent_values)

        # Simple decision based on trend
        if avg_value > 0.5:
            return -0.5  # Counteract
        else:
            return 0.2   # Adjust upward
```

### Multi-Agent Coordination

```python
from std_msgs.msg import Bool

class CoordinatedAIAgent(Node):
    def __init__(self):
        super().__init__('coordinated_ai_agent')

        # Communication with other agents
        self.request_coordination_pub = self.create_publisher(Bool, 'coordination_request', 10)
        self.coordination_response_sub = self.create_subscription(Bool, 'coordination_response', self.coordination_callback, 10)

        # Main functionality
        self.sensor_sub = self.create_subscription(Float32, 'sensor', self.sensor_callback, 10)
        self.command_pub = self.create_publisher(Float32, 'command', 10)

        self.other_agents_ready = True

    def coordination_callback(self, msg):
        """Handle coordination signals from other agents"""
        self.other_agents_ready = msg.data

    def sensor_callback(self, msg):
        """Process sensor and coordinate with other agents if needed"""
        if self.should_coordinate():
            self.request_coordination()
            # Wait for coordination before acting
            if self.other_agents_ready:
                action = self.compute_coordinated_action(msg.data)
            else:
                action = self.compute_individual_action(msg.data)
        else:
            action = self.compute_individual_action(msg.data)

        cmd_msg = Float32()
        cmd_msg.data = action
        self.command_pub.publish(cmd_msg)

    def should_coordinate(self):
        """Determine if coordination is needed"""
        return False  # Simplified for example

    def request_coordination(self):
        """Request coordination with other agents"""
        coord_msg = Bool()
        coord_msg.data = True
        self.request_coordination_pub.publish(coord_msg)
```

## Performance Considerations

When integrating AI agents with ROS 2, consider these performance factors:

### Computational Efficiency

- Use appropriate data structures (NumPy arrays for numerical computations)
- Optimize AI model inference (quantization, pruning, etc.)
- Consider using GPU acceleration when available
- Batch processing when possible

### Real-time Constraints

- Monitor processing times to ensure real-time performance
- Use appropriate QoS settings for time-sensitive data
- Consider different threads for AI processing vs. ROS communication

### Memory Management

- Be mindful of memory usage, especially with large AI models
- Implement proper cleanup of AI model resources
- Monitor memory consumption over time

## Learning Summary

In this chapter, you've learned:

1. How rclpy enables Python AI agents to communicate with ROS 2
2. How to structure AI agents as ROS 2 nodes with proper subscriptions and publications
3. How to integrate popular Python AI libraries (TensorFlow, PyTorch) with ROS 2
4. How to handle various message types for different sensor modalities
5. Advanced patterns like stateful agents and multi-agent coordination
6. Performance considerations when combining AI and robotics

## Exercises

1. Create a Python AI agent that learns to navigate using reinforcement learning
2. Implement an AI agent that processes camera images to detect and track objects
3. Build a multi-agent system where multiple AI agents coordinate to complete a task
4. Integrate a pre-trained neural network with ROS 2 to perform a specific robotics task

## Next Steps

In the next chapter, we'll explore how humanoid robots are modeled in ROS 2 using URDF (Unified Robot Description Format), which is essential for simulation and control of complex robotic systems.