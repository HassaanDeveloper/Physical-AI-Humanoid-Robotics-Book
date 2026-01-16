---
sidebar_position: 4
---

# Assessment: LLM-Based Cognitive Planning

## Learning Objectives
By completing this assessment, students should be able to:
- Explain LLM fundamentals and their role in robotic planning
- Design effective prompts for robotic task generation
- Implement structured output formatting for action sequences
- Create context-aware planning systems with ROS 2 integration

## Assessment Rubric

### Knowledge Level (40 points)
- Define LLM-based cognitive planning and its components (12 points)
- Describe the cognitive planning pipeline stages (15 points)
- List 3 popular LLMs for robotics and their characteristics (13 points)

### Comprehension Level (35 points)
- Explain prompt engineering techniques for robotic planning (15 points)
- Describe structured output formatting requirements (12 points)
- Compare different LLM models for robotic applications (8 points)

### Application Level (25 points)
- Design a complete LLM-based planning system for a navigation task (25 points)

## Practical Exercise

Implement a basic LLM planning system that:

1. **Prompt Design**: Creates effective prompts for a navigation task
2. **Output Parsing**: Validates and parses JSON-formatted action plans
3. **Context Integration**: Incorporates environmental data into planning
4. **ROS 2 Interface**: Designs action server/client for plan execution
5. **Error Handling**: Implements basic error recovery mechanisms

## Coding Challenge

```python
# Implement a function to validate LLM-generated action plans
def validate_action_plan(plan: dict) -> tuple:
    """
    Validate an LLM-generated action plan structure

    Args:
        plan: Dictionary containing the action plan

    Returns:
        tuple: (bool, str) indicating (validity, error_message)
    """
    # Your implementation here
    required_fields = ['plan_id', 'task', 'environment', 'steps']

    # Check required fields
    for field in required_fields:
        if field not in plan:
            return (False, f"Missing required field: {field}")

    # Check steps structure
    for step in plan['steps']:
        step_required = ['step_id', 'action', 'parameters', 'safety_check']
        for field in step_required:
            if field not in step:
                return (False, f"Step missing required field: {field}")

    return (True, "Plan structure is valid")

# Test cases
test_plan_valid = {
    "plan_id": "test_001",
    "task": "navigation",
    "environment": {"location": "lab"},
    "steps": [
        {
            "step_id": 1,
            "action": "move_forward",
            "parameters": {"distance": 1.0},
            "safety_check": "clear_path"
        }
    ]
}

test_plan_invalid = {
    "plan_id": "test_002",
    "task": "manipulation"
    # Missing required fields
}
```

## Answer Key

### Knowledge Level Answers:
1. LLM-based cognitive planning uses large language models to generate robotic action sequences from natural language commands
2. Pipeline stages: Environment Perception → Context Analysis → Task Description → LLM Prompt Engineering → Plan Generation → Structured Output Parsing → ROS 2 Execution → Feedback & Adjustment
3. Popular LLMs:
   - GPT-4: High accuracy, general knowledge, higher cost
   - Claude: Safety-focused, structured output, good for robotics
   - Llama 2: Open-source, customizable, smaller context window

### Comprehension Level Answers:
1. Prompt engineering techniques: Chain-of-Thought, Few-Shot Learning, Role Specification, Constraint Enforcement, Output Formatting
2. Structured output requirements: JSON format with plan_id, task, environment, steps (each with step_id, action, parameters, safety_check)
3. Model comparison: Consider accuracy, response time, cost, context window size, and robotics-specific capabilities

### Application Level Answer:
Student should design a system with:
- Context-rich prompts incorporating environmental data
- JSON schema validation for output parsing
- ROS 2 action server for plan execution
- Feedback loop for plan adjustment
- Comprehensive error handling for LLM output issues