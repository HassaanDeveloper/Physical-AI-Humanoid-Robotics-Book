#!/usr/bin/env python3

import os
import sys
import warnings
# Suppress the deprecation warning for this test
warnings.filterwarnings('ignore', category=FutureWarning)

sys.path.append('.')

# Import just the retrieval part to test if Qdrant is working
try:
    from agent import QdrantRetrievalTool

    print("Testing Qdrant retrieval...")
    retrieval_tool = QdrantRetrievalTool()

    # Test a query to see if we can retrieve content
    chunks = retrieval_tool.retrieve_chunks("ROS 2", limit=3)

    print(f"Retrieved {len(chunks)} chunks for query 'ROS 2'")
    for i, chunk in enumerate(chunks):
        print(f"Chunk {i+1}:")
        print(f"  Score: {chunk['score']:.3f}")
        print(f"  Title: {chunk['metadata'].get('title', 'No title')}")
        print(f"  URL: {chunk['metadata'].get('url', 'No URL')}")
        print(f"  Content preview: {chunk['content'][:100]}...")
        print()

    if chunks:
        print("SUCCESS: Qdrant retrieval is working!")
    else:
        print("WARNING: No chunks retrieved - Qdrant might not have data")

except Exception as e:
    print(f"ERROR with Qdrant retrieval: {e}")
    import traceback
    traceback.print_exc()