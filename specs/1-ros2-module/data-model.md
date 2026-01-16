# Data Model: ROS 2 Module Content Structure

## Content Entity: Chapter
**Definition**: A single educational unit covering specific ROS 2 concepts

**Attributes**:
- id: unique identifier (e.g., "chapter-1-ros2-embodied-intelligence")
- title: display title of the chapter
- position: order in the module sequence
- description: brief overview of chapter content
- learning_outcomes: array of specific skills/knowledge students will gain
- content: main markdown content with lessons and examples
- exercises: hands-on activities for students
- prerequisites: required knowledge or setup steps
- duration: estimated time to complete

**Relationships**:
- Belongs to: ROS 2 Module
- Preceded by: previous chapter (except Chapter 1)
- Followed by: next chapter (except last chapter)

## Content Entity: ROS 2 Module
**Definition**: Educational module containing multiple chapters covering ROS 2 fundamentals

**Attributes**:
- id: "ros2-module"
- title: "ROS 2 Module - The Robotic Nervous System"
- description: Overview of the entire module
- target_audience: "CS/AI students with Python basics, new to robotics"
- total_duration: estimated time to complete all chapters
- learning_outcomes: aggregate outcomes for the entire module

**Relationships**:
- Contains: 4 Chapters
- Associated with: Student entity

## Content Entity: Exercise
**Definition**: Hands-on activity designed to reinforce chapter concepts

**Attributes**:
- id: unique identifier
- title: exercise title
- description: what the exercise aims to teach
- difficulty: beginner, intermediate, advanced
- estimated_time: time to complete
- instructions: step-by-step guide
- expected_outcome: what students should achieve
- code_samples: supporting code snippets

**Relationships**:
- Belongs to: Chapter
- Targets: specific learning outcome

## Content Entity: Code Sample
**Definition**: Executable code example demonstrating ROS 2 concepts

**Attributes**:
- id: unique identifier
- language: programming language (Python, C++, etc.)
- purpose: what concept the code demonstrates
- code: the actual code content
- explanation: description of how the code works
- expected_output: what the code produces when run
- file_path: where the code sample is stored

**Relationships**:
- Used in: Chapter or Exercise
- Demonstrates: specific ROS 2 concept

## Content Entity: Student
**Definition**: Learner engaging with the ROS 2 module content

**Attributes**:
- python_knowledge: basic, intermediate, advanced
- robotics_experience: none, limited, experienced
- learning_pace: self-paced
- progress: track of completed chapters

**Relationships**:
- Accesses: ROS 2 Module and Chapters
- Completes: Exercises