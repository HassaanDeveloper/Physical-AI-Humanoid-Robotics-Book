---
sidebar_position: 6
---

# Assessment: Voice-to-Action Systems

## Learning Objectives
By completing this assessment, students should be able to:
- Implement OpenAI Whisper for voice command processing
- Design command parsing systems with intent recognition
- Integrate voice control with ROS 2 action servers
- Handle noise and errors in real-world voice systems
- Implement feedback mechanisms for command confirmation

## Assessment Rubric

### Knowledge Level (40 points)
- Explain the Voice-to-Action pipeline components (12 points)
- Describe Whisper model sizes and their applications (15 points)
- List 3 noise reduction techniques for voice systems (13 points)

### Comprehension Level (35 points)
- Explain regex-based command parsing patterns (15 points)
- Describe ROS 2 action server/client communication (12 points)
- Compare different feedback mechanisms for voice systems (8 points)

### Application Level (25 points)
- Implement a complete voice command processing system (25 points)

## Practical Exercise

Design and implement a voice-to-action system that:

1. **Speech Recognition**: Uses Whisper to transcribe voice commands
2. **Command Parsing**: Extracts action and parameters using regex
3. **Intent Recognition**: Classifies commands by intent type
4. **ROS 2 Integration**: Connects to action server for execution
5. **Feedback System**: Provides audio and visual confirmation

## Coding Challenge

```python
# Implement a voice command parser with intent recognition
import re

def parse_voice_command(text: str) -> dict:
    """
    Parse voice command and extract action, parameters, and intent

    Args:
        text: Voice command text

    Returns:
        dict: Parsed command structure
    """
    # Remove wake word
    text = re.sub(r'^(robot|assistant|system)[,:\s]+', '', text, flags=re.IGNORECASE)

    # Define command patterns
    patterns = {
        'navigation': r'(move|go|navigate)\s+(forward|backward|left|right)\s*(\d+)?\s*(cm|centimeters|meters)?',
        'rotation': r'(rotate|turn)\s+(left|right)\s*(\d+)?\s*(degrees)?',
        'manipulation': r'(pick|grab|take)\s+(up)?\s+(.+)',
        'query': r'(what|where|how|status|position)\s+(.+)'
    }

    # Check patterns and extract intent
    for intent, pattern in patterns.items():
        match = re.search(pattern, text, re.IGNORECASE)
        if match:
            return {
                'intent': intent,
                'action': intent.split('_')[0],  # navigation -> navigate
                'parameters': match.groups(),
                'raw_text': text,
                'confidence': 0.95  # High confidence for pattern match
            }

    return {
        'intent': 'unknown',
        'action': 'unknown',
        'parameters': [],
        'raw_text': text,
        'confidence': 0.1  # Low confidence
    }

# Test cases
test_commands = [
    "Robot, move forward 50 centimeters",
    "System, turn right 90 degrees",
    "Assistant, pick up the red block",
    "What is your current position?"
]

# Expected outputs:
# 1. intent: 'navigation', action: 'navigate', parameters: ('move', 'forward', '50', 'centimeters')
# 2. intent: 'rotation', action: 'rotate', parameters: ('turn', 'right', '90', 'degrees')
# 3. intent: 'manipulation', action: 'manipulate', parameters: ('pick', 'up', 'the red block')
# 4. intent: 'query', action: 'query', parameters: ('what', 'is your current position?')
```

## Answer Key

### Knowledge Level Answers:
1. Voice-to-Action pipeline: Audio Input → Speech Recognition → Command Parsing → Intent Recognition → Action Generation → ROS 2 Execution → Feedback
2. Whisper model sizes:
   - tiny (39M params, ~1GB VRAM): Fast prototyping
   - base (74M params, ~1GB VRAM): General use
   - small (244M params, ~2GB VRAM): Better accuracy
   - medium (769M params, ~5GB VRAM): High accuracy
   - large (1550M params, ~10GB VRAM): Best accuracy
3. Noise reduction techniques: Audio preprocessing, directional microphones, bandpass filtering

### Comprehension Level Answers:
1. Regex patterns use groups to capture action types, directions, distances, and objects for parameter extraction
2. ROS 2 action communication provides goal request, feedback during execution, and final result reporting
3. Feedback mechanisms include audio confirmation, visual indicators, haptic feedback, and status updates

### Application Level Answer:
Student should implement a system with:
- Whisper transcription with appropriate model size
- Robust regex parsing for command extraction
- Intent classification with keyword matching
- ROS 2 action server for command execution
- Multi-modal feedback (audio + visual)
- Error handling for unrecognized commands