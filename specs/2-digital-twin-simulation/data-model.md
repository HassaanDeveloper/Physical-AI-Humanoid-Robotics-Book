# Data Model: Digital Twin Simulation for Humanoid Robotics

## Overview
This document defines the key data structures and relationships for Module 2 of "Physical AI & Humanoid Robotics" focused on digital twin simulation for humanoid robots. This includes educational content organized for Docusaurus documentation.

## Core Entities

### Digital Twin Model
**Description**: Virtual representation of a physical robot that mirrors its state and behavior in real-time

**Fields**:
- id: Unique identifier for the digital twin
- name: Human-readable name for the twin
- robot_type: Type of robot (e.g., humanoid, wheeled, manipulator)
- physical_robot_id: Reference to the physical robot (if applicable)
- creation_date: Timestamp when the twin was created
- last_updated: Timestamp of last state update
- status: Current operational status (active, paused, maintenance)

**Relationships**:
- One-to-many with SimulationState (has many states over time)
- One-to-many with SensorData (has many sensor readings)

### Simulation Environment
**Description**: Virtual space where physics, sensors, and robot interactions are modeled

**Fields**:
- id: Unique identifier for the environment
- name: Human-readable name
- description: Brief description of the environment
- physics_properties: Configuration for physics simulation
- terrain_data: Information about the environment's terrain
- lighting_conditions: Environmental lighting configuration
- creation_date: When the environment was created
- last_modified: When the environment was last changed

**Relationships**:
- One-to-many with RobotInstance (hosts many robots)
- One-to-many with SimulationEvent (records many events)

### Sensor Simulation
**Description**: Virtual implementation of physical sensors that produces realistic data streams

**Fields**:
- id: Unique identifier for the sensor
- sensor_type: Type of sensor (LiDAR, depth_camera, IMU, etc.)
- robot_id: Reference to the robot carrying the sensor
- position: 3D position on the robot
- orientation: Orientation relative to the robot
- specifications: Technical specs of the sensor
- noise_model: Parameters for realistic noise simulation
- update_rate: Frequency of sensor data updates

**Relationships**:
- Many-to-one with RobotInstance (belongs to one robot)
- One-to-many with SensorData (produces many data points)

### Student Learning Path
**Description**: Structured sequence of modules and exercises designed to achieve learning objectives

**Fields**:
- id: Unique identifier for the learning path
- name: Name of the path
- description: Overview of the learning objectives
- modules: List of modules in the path
- prerequisites: Required knowledge before starting
- estimated_duration: Expected time to complete
- difficulty_level: Beginner, intermediate, advanced

**Relationships**:
- One-to-many with LearningModule (contains many modules)
- One-to-many with StudentProgress (tracks progress for many students)

### Docusaurus Chapter
**Description**: A chapter in the Docusaurus documentation system for the digital twin simulation module

**Fields**:
- id: Unique identifier for the chapter
- title: Title of the chapter
- slug: URL-friendly identifier
- order: Order in which the chapter appears in the documentation
- content_path: Path to the Markdown file
- learning_objectives: List of objectives covered in the chapter
- duration_estimate: Estimated time to complete the chapter
- prerequisites: Prerequisites for this chapter
- associated_exercises: List of exercises related to this chapter

**Relationships**:
- One-to-many with LearningModule (contains many modules)
- Many-to-one with StudentLearningPath (part of one learning path)

## Supporting Entities

### Robot Instance
**Description**: An instantiation of a robot model within a simulation environment

**Fields**:
- id: Unique identifier
- model_reference: Reference to the robot model
- environment_id: Reference to the environment it's in
- position: Current position in the environment
- orientation: Current orientation
- state: Current operational state
- joint_positions: Current positions of all joints (for humanoid robots)

**Relationships**:
- Many-to-one with SimulationEnvironment (exists in one environment)
- One-to-many with SensorSimulation (has many sensors)
- One-to-many with SimulationState (has many state snapshots)

### Sensor Data
**Description**: Data produced by sensor simulations

**Fields**:
- id: Unique identifier
- sensor_id: Reference to the sensor that produced the data
- timestamp: When the data was recorded
- data_payload: The actual sensor data (varies by sensor type)
- frame_id: Coordinate frame for the data
- quality_metrics: Measures of data quality/noise

**Relationships**:
- Many-to-one with SensorSimulation (created by one sensor)

### Simulation State
**Description**: Snapshot of the simulation at a particular point in time

**Fields**:
- id: Unique identifier
- robot_id: Reference to the robot whose state is captured
- timestamp: When the state was recorded
- position: 3D position of the robot
- orientation: Orientation of the robot
- velocities: Linear and angular velocities
- joint_states: Positions and velocities of all joints
- environment_state: State of environment objects

**Relationships**:
- Many-to-one with RobotInstance (state of one robot)

### Learning Module
**Description**: Individual component of a learning path

**Fields**:
- id: Unique identifier
- title: Title of the module
- content: Educational content of the module
- learning_objectives: Specific objectives of the module
- duration: Estimated time to complete
- difficulty: Difficulty level
- prerequisites: Prerequisites for this module

**Relationships**:
- Many-to-one with StudentLearningPath (part of one path)
- One-to-many with Exercise (contains many exercises)

### Exercise
**Description**: Hands-on activity within a learning module

**Fields**:
- id: Unique identifier
- title: Title of the exercise
- description: Detailed description
- instructions: Step-by-step instructions
- expected_outcomes: What students should achieve
- evaluation_criteria: How the exercise will be assessed
- simulation_scenario: Associated simulation scenario

**Relationships**:
- Many-to-one with LearningModule (part of one module)

## Docusaurus-Specific Entities

### Documentation Navigation Item
**Description**: An item in the Docusaurus sidebar navigation

**Fields**:
- id: Unique identifier
- label: Text displayed in the navigation
- type: Type of navigation item (doc, link, category)
- doc_id: Reference to a document if type is 'doc'
- href: External URL if type is 'link'
- items: Child navigation items if type is 'category'
- collapsible: Whether the category can be collapsed
- collapsed: Default collapse state

**Relationships**:
- One-to-many with DocumentationNavigationItem (parent-child relationship for categories)

### Docusaurus Configuration
**Description**: Configuration settings for the Docusaurus documentation site

**Fields**:
- site_title: Title of the site
- site_tagline: Tagline for the site
- favicon: Path to favicon file
- organization_name: GitHub organization name for deployment
- project_name: GitHub project name for deployment
- trailing_slash: Whether to append trailing slashes to URLs
- theme_config: Theme-specific configuration options
- presets: List of Docusaurus presets to use
- themes: List of additional themes
- plugins: List of Docusaurus plugins

## Validation Rules

### Digital Twin Model
- id must be unique
- name must not be empty
- status must be one of: active, paused, maintenance
- creation_date must be before last_updated

### Simulation Environment
- name must be unique within the system
- physics_properties must include gravity, friction coefficients
- terrain_data must be valid 3D representation

### Sensor Simulation
- sensor_type must be one of: LiDAR, depth_camera, IMU, camera, gps, compass
- position and orientation must be within robot bounds
- update_rate must be positive

### Student Learning Path
- estimated_duration must be positive
- difficulty_level must be one of: beginner, intermediate, advanced

### Docusaurus Chapter
- slug must be URL-friendly (alphanumeric, hyphens, underscores only)
- order must be a positive integer
- content_path must exist and be a valid Markdown file
- duration_estimate must be positive

### Documentation Navigation Item
- label must not be empty
- type must be one of: doc, link, category
- If type is 'doc', doc_id must reference an existing document
- If type is 'link', href must be a valid URL

## State Transitions

### Digital Twin Status Transitions
- inactive → active (when simulation starts)
- active → paused (when simulation pauses)
- paused → active (when simulation resumes)
- * → maintenance (when maintenance required)
- maintenance → active (when maintenance complete)

### Simulation Environment States
- created → configured → running → paused → stopped

### Docusaurus Chapter States
- draft → review → published → archived