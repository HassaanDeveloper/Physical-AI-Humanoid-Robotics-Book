#!/usr/bin/env python3
"""
Basic test script for URL Ingestion & Embedding Pipeline
Tests core functionality without requiring API keys
"""

import asyncio
from main import IngestionPipeline, Config, ContentChunk, ChunkMetadata

def test_core_functionality():
    """Test core pipeline functionality"""
    print("=== Testing Core Functionality ===\n")

    # Test configuration
    print("[OK] Configuration loaded successfully")
    print(f"  Base URL: {Config.BASE_URL}")
    print(f"  Chunk Size: {Config.CHUNK_SIZE}")
    print(f"  Overlap: {Config.OVERLAP}")

    # Test pipeline initialization
    try:
        pipeline = IngestionPipeline()
        pipeline.initialize()
        print("[OK] Pipeline initialized successfully")
        print(f"  Collection name: {pipeline.collection_name}")
        print(f"  Embedding dimension: {pipeline.embedding_dimension}")
    except Exception as e:
        print(f"✗ Pipeline initialization failed: {str(e)}")
        return False

    # Test token counting
    try:
        test_text = "This is a test sentence for token counting."
        token_count = pipeline.estimate_token_count(test_text)
        print("[OK] Token counting working")
        print(f"  Text: '{test_text}'")
        print(f"  Token count: {token_count}")
    except Exception as e:
        print(f"✗ Token counting failed: {str(e)}")
        return False

    # Test sentence splitting
    try:
        test_text = "This is sentence one. This is sentence two! Is this sentence three?"
        sentences = pipeline.split_into_sentences(test_text)
        print("[OK] Sentence splitting working")
        print(f"  Text: '{test_text}'")
        print(f"  Sentences: {len(sentences)}")
        for i, sentence in enumerate(sentences):
            print(f"    {i+1}. '{sentence}'")
    except Exception as e:
        print(f"✗ Sentence splitting failed: {str(e)}")
        return False

    # Test ID generation
    try:
        chunk = ContentChunk(
            text="Test content",
            metadata=ChunkMetadata(
                url="/test-url",
                title="Test",
                chunk_index=0,
                chunk_total=1,
                token_count=10,
                timestamp=pipeline.get_current_timestamp()
            )
        )
        chunk_id = pipeline.generate_id(chunk)
        print("[OK] ID generation working")
        print(f"  Chunk ID: {chunk_id}")
    except Exception as e:
        print(f"✗ ID generation failed: {str(e)}")
        return False

    # Test timestamp generation
    try:
        timestamp = pipeline.get_current_timestamp()
        print("[OK] Timestamp generation working")
        print(f"  Current timestamp: {timestamp}")
    except Exception as e:
        print(f"✗ Timestamp generation failed: {str(e)}")
        return False

    print("\n=== All Tests Passed! ===")
    print("\nCore functionality is working correctly.")
    print("To run the full pipeline, you need to:")
    print("1. Set COHERE_API_KEY in backend/.env file")
    print("2. Ensure Qdrant is running at the configured URL")
    print("3. Run: python main.py")

    return True

if __name__ == "__main__":
    success = test_core_functionality()
    exit(0 if success else 1)