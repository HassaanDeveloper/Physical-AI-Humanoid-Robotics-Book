---
id: chapter-4-humanoid-modeling-urdf
title: Chapter 4 - Humanoid Modeling with URDF
sidebar_label: Humanoid Modeling with URDF
sidebar_position: 4
description: Understanding humanoid robot structure using URDF (Unified Robot Description Format)
tags: [ros2, urdf, humanoid, robot-modeling, robotics]
keywords: [URDF, robot modeling, humanoid, robotics, robot description]
learning_outcomes:
  - Understand the structure and components of URDF files
  - Interpret URDF files for humanoid robot models
  - Identify joints, links, and their relationships in URDF
  - Understand kinematic properties of humanoid robots through URDF
prerequisites:
  - Understanding of ROS 2 concepts (Chapter 1-3)
  - Basic knowledge of 3D geometry and kinematics
duration_minutes: 60
---

# Chapter 4 - Humanoid Modeling with URDF

## Overview

In this final chapter of our ROS 2 module, we'll explore how humanoid robots are represented in ROS 2 using URDF (Unified Robot Description Format). URDF is an XML-based format that describes robot models including their physical structure, joints, and kinematic properties. Understanding URDF is crucial for working with humanoid robots in simulation and real-world applications.

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines:

- **Links**: Rigid parts of the robot (e.g., arms, legs, torso)
- **Joints**: Connections between links that allow motion
- **Visual and collision properties**: How the robot appears and interacts physically
- **Inertial properties**: Mass, center of mass, and moments of inertia
- **Transmission interfaces**: How joints connect to actuators

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Joints define connections between links -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.5 -0.1" rpy="0 0 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Links in URDF

Links represent rigid parts of the robot. Each link can have multiple properties:

### Visual Properties
Defines how the link appears in visualization tools:

```xml
<link name="link_name">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Box, cylinder, sphere, or mesh -->
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>
</link>
```

### Collision Properties
Defines how the link interacts in physics simulations:

```xml
<link name="link_name">
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>
</link>
```

### Inertial Properties
Defines physical properties for dynamics simulation:

```xml
<link name="link_name">
  <inertial>
    <mass value="0.1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>
```

## Joints in URDF

Joints define how links connect and move relative to each other. There are several joint types:

### Fixed Joint
No movement allowed:

```xml
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>
```

### Revolute Joint
Rotational movement around a single axis with limits:

```xml
<joint name="revolute_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
</joint>
```

### Continuous Joint
Rotational movement without limits:

```xml
<joint name="continuous_joint" type="continuous">
  <parent link="base"/>
  <child link="wheel"/>
  <origin xyz="0 0.2 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

### Prismatic Joint
Linear sliding movement along an axis:

```xml
<joint name="prismatic_joint" type="prismatic">
  <parent link="base"/>
  <child link="slider"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="0.5" effort="10" velocity="1"/>
</joint>
```

## Humanoid Robot URDF Structure

Humanoid robots have a characteristic structure with multiple limbs. A typical humanoid URDF includes:

### Torso and Head
The central part of the robot with head attached:

```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </visual>
</link>

<joint name="torso_to_head" type="revolute">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.25" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.5" upper="0.5" effort="5" velocity="2"/>
</joint>

<link name="head">
  <visual>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
  </visual>
</link>
```

### Arms
Symmetric left and right arms with multiple joints:

```xml
<!-- Right arm -->
<joint name="right_shoulder_pitch" type="revolute">
  <parent link="torso"/>
  <child link="right_upper_arm"/>
  <origin xyz="-0.15 0 0.1" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
</joint>

<link name="right_upper_arm">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
  </visual>
</link>

<joint name="right_elbow" type="revolute">
  <parent link="right_upper_arm"/>
  <child link="right_lower_arm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-1.57" upper="0" effort="10" velocity="1"/>
</joint>

<link name="right_lower_arm">
  <visual>
    <geometry>
      <cylinder radius="0.04" length="0.25"/>
    </geometry>
  </visual>
</link>
```

### Legs
Symmetric left and right legs for bipedal locomotion:

```xml
<!-- Right leg -->
<joint name="right_hip" type="revolute">
  <parent link="torso"/>
  <child link="right_thigh"/>
  <origin xyz="0.1 0 -0.2" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-0.5" upper="1.0" effort="20" velocity="1"/>
</joint>

<link name="right_thigh">
  <visual>
    <geometry>
      <cylinder radius="0.06" length="0.4"/>
    </geometry>
  </visual>
</link>

<joint name="right_knee" type="revolute">
  <parent link="right_thigh"/>
  <child link="right_shin"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="1.57" effort="20" velocity="1"/>
</joint>

<link name="right_shin">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.4"/>
    </geometry>
  </visual>
</link>
```

## URDF Tools and Visualization

Several tools help work with URDF files:

### Checking URDF Syntax
```bash
# Validate URDF file
check_urdf robot.urdf

# Display robot information
urdf_to_graphiz robot.urdf
```

### Visualization
```bash
# Visualize robot in RViz
ros2 run rviz2 rviz2

# Launch robot in Gazebo simulation
ros2 launch gazebo_ros spawn_entity.launch.py entity:=robot_name
```

## Kinematic Chains and Forward Kinematics

URDF defines kinematic chains that allow computation of forward kinematics - determining the position and orientation of end-effectors based on joint angles.

### Denavit-Hartenberg Parameters
While not explicitly in URDF, the joint relationships define the kinematic structure:

```xml
<!-- Example of a simple kinematic chain: shoulder -> elbow -> wrist -->
<joint name="shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0 0.2 0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>

<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

## Working with URDF in Python

Using Python to work with URDF and robot models:

### Loading URDF Programmatically

```python
import rclpy
from rclpy.node import Node
from urdf_parser_py.urdf import URDF
import os

class URDFAnalyzer(Node):
    def __init__(self):
        super().__init__('urdf_analyzer')

        # Load URDF from file
        robot_file = os.path.join(os.getenv('HOME'), 'robot.urdf')
        with open(robot_file, 'r') as f:
            robot_desc = f.read()

        # Parse URDF
        robot = URDF.from_xml_string(robot_desc)

        # Analyze robot structure
        self.analyze_robot(robot)

    def analyze_robot(self, robot):
        """Analyze robot structure and print information"""
        self.get_logger().info(f'Robot name: {robot.name}')
        self.get_logger().info(f'Number of links: {len(robot.links)}')
        self.get_logger().info(f'Number of joints: {len(robot.joints)}')

        for link in robot.links:
            self.get_logger().info(f'Link: {link.name}')

        for joint in robot.joints:
            self.get_logger().info(f'Joint: {joint.name} ({joint.type}) connects {joint.parent} to {joint.child}')

def main(args=None):
    rclpy.init(args=args)
    analyzer = URDFAnalyzer()

    # Print robot info once
    rclpy.spin_once(analyzer, timeout_sec=1)
    analyzer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Working with TF Trees

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer

class URDFTransformMonitor(Node):
    def __init__(self):
        super().__init__('urdf_transform_monitor')

        # Create TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_link_pose(self, target_frame, source_frame='base_link'):
        """Get pose of a link relative to another frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                source_frame,
                target_frame,
                rclpy.time.Time()
            )
            return transform.transform
        except Exception as e:
            self.get_logger().error(f'Could not lookup transform: {e}')
            return None
```

## Best Practices for URDF

### Naming Conventions
- Use consistent, descriptive names
- Use underscores for multi-word names
- Prefix symmetric parts (left_/right_)

### Units
- Use meters for distances
- Use radians for angles
- Use kilograms for mass

### Organization
- Use xacro macros for repetitive structures
- Organize files by function (arms.urdf, legs.urdf, etc.)
- Include proper inertial properties for simulation

## Learning Summary

In this chapter, you've learned:

1. The structure and components of URDF files
2. How to define links with visual, collision, and inertial properties
3. Different joint types and their characteristics
4. How humanoid robots are structured in URDF
5. Tools for validating and visualizing URDF models
6. How to work with URDF programmatically in Python
7. Best practices for creating effective robot models

## Exercises

1. Create a simple URDF model of a 2-wheeled robot
2. Modify an existing URDF file to add additional degrees of freedom
3. Write a Python script that parses a URDF file and prints the kinematic chain
4. Create a URDF model of a simple humanoid robot with basic arm and leg structures

## Conclusion

With this final chapter, you have completed the ROS 2 Module - The Robotic Nervous System. You now understand:

- How ROS 2 serves as middleware connecting AI agents to humanoid robots
- The core communication patterns in ROS 2 (nodes, topics, services)
- How to bridge Python AI agents with ROS 2 using rclpy
- How humanoid robots are modeled using URDF

These foundational concepts provide the groundwork for developing sophisticated robotic applications that combine AI with physical systems.