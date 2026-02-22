#!/usr/bin/env python3
"""
RAG Pipeline Validation - Retrieve and Validate Stored Embeddings from Qdrant

This script connects to Qdrant to retrieve stored embeddings and validates
the end-to-end retrieval pipeline to ensure correct, relevant, and traceable
context is returned for queries.
"""

import os
import sys
import argparse
import logging
from typing import List, Dict, Any, Optional
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere
import numpy as np
from datetime import datetime
import json

# Load environment variables
load_dotenv()

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class RAGValidator:
    """
    RAG Pipeline Validator

    Validates that stored embeddings in Qdrant can be retrieved correctly
    and that the content matches the original source material.
    """

    def __init__(self):
        """Initialize the RAG validator with connections to Qdrant and Cohere."""
        self.qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")
        self.cohere_api_key = os.getenv("COHERE_API_KEY")
        self.collection_name = os.getenv("COLLECTION_NAME", "book_content")

        # Initialize Qdrant client
        if self.qdrant_api_key:
            self.qdrant_client = QdrantClient(
                url=self.qdrant_url,
                api_key=self.qdrant_api_key
            )
        else:
            self.qdrant_client = QdrantClient(host="localhost", port=6333)

        # Initialize Cohere client
        if not self.cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")
        self.cohere_client = cohere.Client(self.cohere_api_key)

        logger.info(f"Initialized RAGValidator with collection: {self.collection_name}")

    def connect_to_qdrant(self) -> bool:
        """
        Test connection to Qdrant and verify collection exists.

        Returns:
            bool: True if connection successful and collection exists
        """
        try:
            # Test connection
            collections = self.qdrant_client.get_collections()
            collection_names = [coll.name for coll in collections.collections]

            if self.collection_name not in collection_names:
                logger.error(f"Collection '{self.collection_name}' not found in Qdrant")
                logger.info(f"Available collections: {collection_names}")
                return False

            # Get collection info
            collection_info = self.qdrant_client.get_collection(self.collection_name)
            logger.info(f"Connected to Qdrant collection '{self.collection_name}'")
            logger.info(f"Collection vectors count: {collection_info.points_count}")

            return True

        except Exception as e:
            logger.error(f"Failed to connect to Qdrant: {str(e)}")
            return False

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for the given text using Cohere.

        Args:
            text: Text to generate embedding for

        Returns:
            List of floats representing the embedding vector
        """
        try:
            response = self.cohere_client.embed(
                texts=[text],
                model="embed-english-v3.0",
                input_type="search_query"  # Using search_query for query embeddings
            )
            return response.embeddings[0]
        except Exception as e:
            logger.error(f"Failed to generate embedding: {str(e)}")
            raise

    def search_similar(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar content in Qdrant based on the query.

        Args:
            query: Query text to search for
            limit: Number of results to return

        Returns:
            List of dictionaries containing retrieved content and metadata
        """
        try:
            query_embedding = self.generate_embedding(query)

            search_results = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=limit,
                with_payload=True,
                with_vectors=False
            ).points

            results = []
            for result in search_results:
                # Extract metadata - check if it's nested or directly in payload
                payload = result.payload
                if 'metadata' in payload and isinstance(payload['metadata'], dict):
                    # Metadata is nested in a 'metadata' field
                    extracted_metadata = payload['metadata']
                else:
                    # Metadata fields are directly in the payload
                    extracted_metadata = {
                        'url': payload.get('url', ''),
                        'title': payload.get('title', ''),
                        'chunk_index': payload.get('chunk_index'),
                        # Include other potential metadata fields
                        **{k: v for k, v in payload.items() if k not in ['text', 'metadata']}
                    }

                results.append({
                    'id': result.id,
                    'score': result.score,
                    'payload': result.payload,
                    'content': payload.get('text', ''),
                    'metadata': extracted_metadata
                })

            logger.info(f"Found {len(results)} results for query: '{query[:50]}...'")
            return results

        except Exception as e:
            logger.error(f"Search failed: {str(e)}")
            return []

    def validate_content_accuracy(self, query: str, retrieved_content: str, source_content: str = None) -> Dict[str, Any]:
        """
        Validate that retrieved content matches source content.

        Args:
            query: Original query
            retrieved_content: Content retrieved from Qdrant
            source_content: Original source content (if available for comparison)

        Returns:
            Dictionary with validation results
        """
        # For now, we'll calculate a simple similarity score based on common words
        # In a real implementation, we'd compare against known source content

        query_words = set(query.lower().split())
        retrieved_words = set(retrieved_content.lower().split())

        if source_content:
            source_words = set(source_content.lower().split())
            # Compare retrieved vs source
            intersection = retrieved_words.intersection(source_words)
            union = retrieved_words.union(source_words)
            jaccard_similarity = len(intersection) / len(union) if union else 0

            # Check if retrieved content contains important terms from source
            source_coverage = len(intersection) / len(source_words) if source_words else 0
        else:
            # Without source content, just check relevance to query
            intersection = query_words.intersection(retrieved_words)
            jaccard_similarity = len(intersection) / len(query_words) if query_words else 0
            source_coverage = 0

        return {
            'jaccard_similarity': jaccard_similarity,
            'source_coverage': source_coverage,
            'relevant_terms_found': len(intersection),
            'validation_passed': jaccard_similarity > 0.1  # Threshold for relevance
        }

    def validate_metadata_traceability(self, metadata: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate that metadata correctly links to original book pages.

        Args:
            metadata: Metadata dictionary from retrieved content

        Returns:
            Dictionary with validation results
        """
        required_fields = ['url', 'title']
        missing_fields = [field for field in required_fields if field not in metadata]

        url_valid = metadata.get('url', '').startswith(('http://', 'https://'))
        title_present = bool(metadata.get('title', '').strip())

        return {
            'missing_fields': missing_fields,
            'url_valid': url_valid,
            'title_present': title_present,
            'metadata_valid': len(missing_fields) == 0 and url_valid and title_present
        }

    def validate_pipeline(self) -> Dict[str, Any]:
        """
        Validate the entire RAG pipeline with multiple test queries.

        Returns:
            Dictionary with overall validation results
        """
        logger.info("Starting RAG pipeline validation...")

        # Sample test queries based on the book content
        test_queries = [
            "What are Vision-Language-Action (VLA) systems?",
            "Explain the architecture of autonomous humanoid robots",
            "How does digital twin technology work in robotics?",
            "What is Isaac Sim used for?",
            "Explain ROS 2 fundamentals for embodied intelligence",
            "What are the foundations of vision-language-action systems?",
            "How do humanoid robots integrate perception and action?",
            "What are the key components of VLA systems?"
        ]

        all_validation_results = []

        for query in test_queries:
            logger.info(f"Validating query: '{query}'")

            # Search for similar content
            search_results = self.search_similar(query, limit=3)

            if not search_results:
                logger.warning(f"No results found for query: '{query}'")
                continue

            # Validate the first result (highest scoring)
            first_result = search_results[0]
            content = first_result['content']
            metadata = first_result['metadata']

            # Validate content accuracy
            content_validation = self.validate_content_accuracy(query, content)

            # Validate metadata traceability
            metadata_validation = self.validate_metadata_traceability(metadata)

            # Combine results
            validation_result = {
                'query': query,
                'content_validation': content_validation,
                'metadata_validation': metadata_validation,
                'retrieved_content_preview': content[:200] + "..." if len(content) > 200 else content,
                'relevance_score': first_result['score'],
                'metadata': metadata
            }

            all_validation_results.append(validation_result)

        # Calculate overall statistics
        total_queries = len(all_validation_results)
        content_pass_count = sum(1 for r in all_validation_results if r['content_validation']['validation_passed'])
        metadata_pass_count = sum(1 for r in all_validation_results if r['metadata_validation']['metadata_valid'])

        overall_accuracy = content_pass_count / total_queries if total_queries > 0 else 0
        metadata_accuracy = metadata_pass_count / total_queries if total_queries > 0 else 0

        validation_summary = {
            'total_queries_tested': total_queries,
            'content_validation_pass_rate': overall_accuracy,
            'metadata_validation_pass_rate': metadata_accuracy,
            'individual_results': all_validation_results,
            'pipeline_status': 'PASS' if overall_accuracy > 0.5 and metadata_accuracy > 0.8 else 'FAIL',
            'timestamp': datetime.now().isoformat()
        }

        return validation_summary

    def log_validation_results(self, results: Dict[str, Any]):
        """
        Log validation results in a readable format.

        Args:
            results: Dictionary containing validation results
        """
        print("\n" + "="*60)
        print("RAG PIPELINE VALIDATION RESULTS")
        print("="*60)

        print(f"Total queries tested: {results['total_queries_tested']}")
        print(f"Content validation pass rate: {results['content_validation_pass_rate']:.2%}")
        print(f"Metadata validation pass rate: {results['metadata_validation_pass_rate']:.2%}")
        print(f"Overall pipeline status: {results['pipeline_status']}")
        print(f"Validation timestamp: {results['timestamp']}")

        print("\nIndividual Query Results:")
        print("-" * 40)

        for i, result in enumerate(results['individual_results'], 1):
            print(f"\n{i}. Query: '{result['query'][:50]}...'")
            print(f"   Relevance Score: {result['relevance_score']:.3f}")
            print(f"   Content Validation: {'PASS' if result['content_validation']['validation_passed'] else 'FAIL'}")
            print(f"   Metadata Validation: {'PASS' if result['metadata_validation']['metadata_valid'] else 'FAIL'}")
            print(f"   Content Preview: {result['retrieved_content_preview']}")
            print(f"   Source: {result['metadata'].get('url', 'N/A')}")


def main():
    """Main function to run the RAG validation."""
    parser = argparse.ArgumentParser(description='Validate RAG Pipeline Retrieval')
    parser.add_argument('--query', type=str, help='Single query to test')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose logging')

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    try:
        # Initialize validator
        validator = RAGValidator()

        # Test connection to Qdrant
        if not validator.connect_to_qdrant():
            logger.error("Failed to connect to Qdrant or collection not found")
            sys.exit(1)

        if args.query:
            # Test a single query
            logger.info(f"Testing single query: {args.query}")
            results = validator.search_similar(args.query)

            print(f"\nResults for query: '{args.query}'")
            print("-" * 50)

            for i, result in enumerate(results, 1):
                print(f"\n{i}. Score: {result['score']:.3f}")
                print(f"   Content: {result['content'][:200]}...")
                print(f"   URL: {result['metadata'].get('url', 'N/A')}")
                print(f"   Title: {result['metadata'].get('title', 'N/A')}")
        else:
            # Run full validation pipeline
            validation_results = validator.validate_pipeline()
            validator.log_validation_results(validation_results)

            # Exit with error code if pipeline failed
            if validation_results['pipeline_status'] == 'FAIL':
                logger.error("Pipeline validation failed!")
                sys.exit(1)
            else:
                logger.info("Pipeline validation completed successfully!")

    except Exception as e:
        logger.error(f"Error during validation: {str(e)}")
        sys.exit(1)


if __name__ == "__main__":
    main()