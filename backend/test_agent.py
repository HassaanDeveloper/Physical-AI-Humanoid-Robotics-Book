#!/usr/bin/env python3

import sys
import warnings
# Suppress the deprecation warning for this test
warnings.filterwarnings('ignore', category=FutureWarning)

sys.path.append('.')

from agent import GeminiBookAgent

try:
    agent = GeminiBookAgent()
    print('Agent initialized successfully')
    result = agent.ask('What is ROS 2?', max_chunks=3)
    print('Query processed successfully')
    print('Response:', result['response'][:500] + '...' if len(result['response']) > 500 else result['response'])
    print('Status:', result['status'])
    print()
    print('Retrieved chunks:', len(result['retrieved_chunks']))
    for i, chunk in enumerate(result['retrieved_chunks']):
        print(f'  Chunk {i+1}: Score={chunk["score"]:.3f}, Title="{chunk["metadata"]["title"]}"')
except Exception as e:
    print(f'Error: {e}')
    import traceback
    traceback.print_exc()