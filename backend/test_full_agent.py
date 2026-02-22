#!/usr/bin/env python3

import os
import sys
import warnings
# Suppress the deprecation warning for this test
warnings.filterwarnings('ignore', category=FutureWarning)

sys.path.append('.')

try:
    from agent import GeminiBookAgent

    print("Initializing agent...")
    agent = GeminiBookAgent()
    print("Agent initialized successfully")

    print("\nTesting query: 'What is ROS 2?'")
    result = agent.ask('What is ROS 2?', max_chunks=3)

    print(f"Status: {result['status']}")
    print(f"Response: {result['response'][:1000] + '...' if len(result['response']) > 1000 else result['response']}")

    print(f"\nRetrieved {len(result['retrieved_chunks'])} chunks:")
    for i, chunk in enumerate(result['retrieved_chunks']):
        print(f"  {i+1}. Score: {chunk['score']:.3f} | Title: {chunk['metadata']['title'][:60]}...")

except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()