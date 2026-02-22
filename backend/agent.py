#!/usr/bin/env python3
"""
Gemini Agent with Retrieval Integration

This script creates a Gemini agent that integrates with Qdrant retrieval
to provide grounded, context-aware responses from the book content.
The agent enforces strict grounding constraints to prevent hallucination.
"""

import os
import sys
import argparse
import logging
import asyncio
from typing import List, Dict, Any, Optional
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere
import google.genai as genai
import json
from datetime import datetime

# Load environment variables
load_dotenv()

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class QdrantRetrievalTool:
    """
    Qdrant Retrieval Tool

    Provides vector similarity search functionality to retrieve relevant
    content chunks from the Qdrant database.
    """

    def __init__(self):
        """Initialize the Qdrant retrieval tool with connections."""
        self.qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")
        self.cohere_api_key = os.getenv("COHERE_API_KEY")
        self.gemini_api_key = os.getenv("GEMINI_API_KEY")
        self.collection_name = os.getenv("COLLECTION_NAME", "book_content")

        # Initialize Qdrant client
        if self.qdrant_api_key:
            self.qdrant_client = QdrantClient(
                url=self.qdrant_url,
                api_key=self.qdrant_api_key
            )
        else:
            self.qdrant_client = QdrantClient(host="localhost", port=6333)

        # Initialize Cohere client for query embeddings (required for Qdrant compatibility)
        # The Qdrant collection was created with Cohere embeddings, so we must use Cohere for query embeddings
        if not self.cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is required for embedding generation")
        self.cohere_client = cohere.Client(self.cohere_api_key)
        logger.info("Using Cohere for embeddings (required for Qdrant compatibility)")

        logger.info(f"Initialized QdrantRetrievalTool with collection: {self.collection_name}")

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for the given text using either Gemini or Cohere.

        Args:
            text: Text to generate embedding for

        Returns:
            List of floats representing the embedding vector
        """
        try:
            # Use Cohere for embeddings (required for Qdrant compatibility)
            response = self.cohere_client.embed(
                texts=[text],
                model="embed-english-v3.0",
                input_type="search_query"  # Using search_query for query embeddings
            )
            return response.embeddings[0]
        except Exception as e:
            logger.error(f"Failed to generate embedding: {str(e)}")
            raise

    def retrieve_chunks(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve relevant content chunks from Qdrant based on the query.

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
                    'content': payload.get('text', ''),
                    'metadata': extracted_metadata,
                    'source_url': extracted_metadata.get('url', ''),
                    'title': extracted_metadata.get('title', '')
                })

            logger.info(f"Retrieved {len(results)} chunks for query: '{query[:50]}...'")
            return results

        except Exception as e:
            logger.error(f"Retrieval failed: {str(e)}")
            return []


class GeminiBookAgent:
    """
    Gemini Book Agent with Retrieval Integration

    An AI agent that uses Google's Gemini API and integrates with Qdrant retrieval
    to provide grounded responses based on book content.
    """

    def __init__(self):
        """Initialize the Gemini agent with retrieval capabilities."""
        self.gemini_api_key = os.getenv("GEMINI_API_KEY")
        if not self.gemini_api_key:
            raise ValueError("GEMINI_API_KEY environment variable is required")

        self.client = genai.Client()
        self.model_name = 'gemini-pro' # Hardcode for now
        
        logger.info(f"Initialized with model: {self.model_name}")
        self.retrieval_tool = QdrantRetrievalTool()

        logger.info("Initialized GeminiBookAgent with retrieval tool")
        self.greetings = ["hi", "hello", "hey", "good morning", "good afternoon", "good evening"]

    def _is_greeting(self, query: str) -> bool:
        """
        Check if the query is a simple greeting.

        Args:
            query: The user's query.

        Returns:
            True if the query is a greeting, False otherwise.
        """
        return query.lower() in self.greetings

    def evaluate_context_sufficiency(self, query: str, retrieved_chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Evaluate whether the retrieved context is sufficient to answer the query.

        Args:
            query: The original query
            retrieved_chunks: List of retrieved content chunks

        Returns:
            Dictionary with evaluation results
        """
        # Basic sufficiency evaluation based on relevance scores and content coverage
        if not retrieved_chunks:
            return {
                'sufficient': False,
                'confidence': 0.0,
                'reason': 'No relevant content found in the book',
                'relevance_score': 0.0
            }

        # Calculate average relevance score
        avg_score = sum(chunk['score'] for chunk in retrieved_chunks) / len(retrieved_chunks)

        # Check if the highest-scoring chunk contains relevant information
        highest_scoring_chunk = max(retrieved_chunks, key=lambda x: x['score'])

        # Simple relevance check - if score is above threshold, consider sufficient
        # Lower the threshold to allow more responses, especially for technical terms that might have lower scores
        threshold = 0.01  # Very low threshold to allow responses even with weak matches
        is_sufficient = avg_score > threshold or max(chunk['score'] for chunk in retrieved_chunks) > 0.05

        return {
            'sufficient': is_sufficient,
            'confidence': avg_score,
            'reason': f'Average relevance score: {avg_score:.3f}',
            'relevance_score': avg_score
        }

    def check_for_hallucination(self, response: str, retrieved_chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Check if the agent's response contains hallucinated information.

        Args:
            response: The agent's response
            retrieved_chunks: List of retrieved content chunks used to generate response

        Returns:
            Dictionary with hallucination check results
        """
        # For now, implement a basic check by ensuring response content
        # relates to the retrieved content. In practice, this would be more sophisticated.

        response_words = set(response.lower().split())

        # Aggregate all content from retrieved chunks
        all_retrieved_content = ' '.join([chunk['content'] for chunk in retrieved_chunks])
        retrieved_words = set(all_retrieved_content.lower().split())

        # Calculate overlap
        overlap = response_words.intersection(retrieved_words)
        overlap_ratio = len(overlap) / len(response_words) if response_words else 0

        # Check if response contains content not in retrieved chunks
        unique_to_response = response_words.difference(retrieved_words)
        unique_ratio = len(unique_to_response) / len(response_words) if response_words else 0

        # Consider hallucination if more than 30% of words are unique to response
        has_hallucination = unique_ratio > 0.3

        return {
            'has_hallucination': has_hallucination,
            'overlap_ratio': overlap_ratio,
            'unique_ratio': unique_ratio,
            'words_not_in_retrieved': list(unique_to_response)[:10],  # First 10 unique words
            'evaluation_confidence': 0.8  # Confidence in hallucination check
        }

    def generate_grounded_response(self, query: str, retrieved_chunks: List[Dict[str, Any]]) -> str:
        """
        Generate a response grounded in the retrieved content.

        Args:
            query: The original query
            retrieved_chunks: List of retrieved content chunks

        Returns:
            Grounded response string
        """
        if not retrieved_chunks:
            return "I cannot answer that question as it's not covered in the book content."

        # Format the retrieved content for context
        formatted_context = "\n\n".join([
            f"Source: {chunk['metadata'].get('title', 'Unknown Title')} ({chunk['metadata'].get('url', 'No URL')})\n"
            f"Content: {chunk['content'][:500]}..."  # Limit content length
            for chunk in retrieved_chunks
        ])

        # Prepare the prompt to enforce grounding
        full_prompt = f"""You are an AI assistant that answers questions based ONLY on the provided book content.
        Do not use any external knowledge. If the question cannot be answered using the provided content,
        clearly state that the information is not available in the book.

        Book Content Context:
        {formatted_context}

        Question: {query}

        Answer:"""

        try:
            response = self.client.models.generate_content(
                model=f"models/{self.model_name}",
                contents=full_prompt
            )

            # Handle potential response format differences
            if hasattr(response, 'text'):
                return response.text
            elif hasattr(response, 'candidates') and response.candidates:
                candidate = response.candidates[0]
                if hasattr(candidate, 'content') and hasattr(candidate.content, 'parts'):
                    return ''.join([part.text for part in candidate.content.parts if hasattr(part, 'text')])
                else:
                    return str(response)
            else:
                return str(response)

        except Exception as e:
            logger.error(f"Error generating response: {str(e)}")
            # Return fallback response with the retrieved content
            return self.get_fallback_response(query, retrieved_chunks)

    def ask(self, query: str, max_chunks: int = 5) -> Dict[str, Any]:
        """
        Process a query using the Gemini agent with retrieval integration.

        Args:
            query: The query to process
            max_chunks: Maximum number of chunks to retrieve

        Returns:
            Dictionary with the agent's response and metadata
        """
        logger.info(f"Processing query: '{query[:50]}...'")

        if self._is_greeting(query):
            return {
                'query': query,
                'response': "Hello! I'm your book assistant. How can I help you today?",
                'retrieved_chunks': [],
                'context_evaluation': {'sufficient': True, 'confidence': 1.0, 'reason': 'Greeting'},
                'timestamp': datetime.now().isoformat(),
                'status': 'greeting'
            }

        # Step 1: Retrieve relevant chunks
        retrieved_chunks = self.retrieval_tool.retrieve_chunks(query, limit=max_chunks)

        # Step 2: Evaluate context sufficiency
        context_evaluation = self.evaluate_context_sufficiency(query, retrieved_chunks)

        # Step 3: Generate response based on retrieved content
        response = self.generate_grounded_response(query, retrieved_chunks)

        # Check if the response indicates insufficient context
        if ("cannot adequately answer" in response.lower() or
            "not covered in the book" in response.lower() or
            "not available in the book" in response.lower() or
            "not found in the book" in response.lower()):
            # If the AI says it can't answer, return the raw retrieved content
            fallback_response = self.get_fallback_response(query, retrieved_chunks)
            response = fallback_response
            status = 'context_used_with_ai_failure'
        else:
            status = 'success'

        # Step 4: Check for hallucination if we have a proper response
        if status == 'success' and context_evaluation['sufficient']:
            hallucination_check = self.check_for_hallucination(response, retrieved_chunks)
            result = {
                'query': query,
                'response': response,
                'retrieved_chunks': retrieved_chunks,
                'context_evaluation': context_evaluation,
                'hallucination_check': hallucination_check,
                'timestamp': datetime.now().isoformat(),
                'status': status
            }

            if hallucination_check['has_hallucination']:
                logger.warning(f"Potential hallucination detected in response to query: '{query[:30]}...'")
                result['warning'] = 'Response may contain information not found in retrieved content'
        else:
            result = {
                'query': query,
                'response': response,
                'retrieved_chunks': retrieved_chunks,
                'context_evaluation': context_evaluation,
                'timestamp': datetime.now().isoformat(),
                'status': status
            }

        return result

    # Add a fallback method to return retrieved content when API fails
    def get_fallback_response(self, query: str, retrieved_chunks: List[Dict[str, Any]]) -> str:
        """
        Fallback response when API call fails - return the most relevant retrieved content.

        Args:
            query: The original query
            retrieved_chunks: List of retrieved content chunks

        Returns:
            Fallback response based on retrieved content
        """
        if not retrieved_chunks:
            return "I cannot answer that question as it's not covered in the book content."

        # Sort chunks by relevance score and return the most relevant one(s)
        sorted_chunks = sorted(retrieved_chunks, key=lambda x: x['score'], reverse=True)

        # Build response from the most relevant chunks
        if sorted_chunks:
            # Return only the content of the most relevant chunk
            return sorted_chunks[0]['content']
        else:
            return "I cannot answer that question as it's not covered in the book content."

    def conversation(self, messages: List[Dict[str, str]], max_chunks: int = 5) -> Dict[str, Any]:
        """
        Process a conversation with multiple messages using retrieval integration.

        Args:
            messages: List of messages in the conversation (role and content)
            max_chunks: Maximum number of chunks to retrieve

        Returns:
            Dictionary with the agent's response and metadata
        """
        # Extract the latest user query
        latest_user_query = ""
        for message in reversed(messages):
            if message['role'] == 'user':
                latest_user_query = message['content']
                break

        if not latest_user_query:
            return {
                'response': "No user query found in the conversation messages.",
                'status': 'error'
            }

        # Process using the same logic as single query
        return self.ask(latest_user_query, max_chunks)


def main():
    """Main function to run the Gemini agent."""
    parser = argparse.ArgumentParser(description='Gemini Agent with Qdrant Retrieval')
    parser.add_argument('--query', type=str, help='Single query to process')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose logging')

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    try:
        # Initialize the agent
        agent = GeminiBookAgent()

        if args.query:
            # Process a single query
            logger.info(f"Processing single query: {args.query}")
            result = agent.ask(args.query)

            print(f"\nQuery: {result['query']}")
            print(f"Response: {result['response']}")
            print(f"Status: {result['status']}")

            if result['status'] != 'insufficient_context':
                print(f"Context Sufficiency: {result['context_evaluation']['reason']}")
                if 'hallucination_check' in result:
                    print(f"Hallucination Check: {'Passed' if not result['hallucination_check']['has_hallucination'] else 'Flagged'}")

            # Show retrieved sources
            print("\nRetrieved Sources:")
            for i, chunk in enumerate(result['retrieved_chunks'][:3], 1):  # Show first 3 sources
                print(f"  {i}. {chunk['metadata'].get('title', 'No Title')}")
                print(f"     URL: {chunk['metadata'].get('url', 'No URL')}")
                print(f"     Relevance: {chunk['score']:.3f}")
        else:
            # Interactive mode
            print("Gemini Book Agent started. Type 'quit' to exit.")
            print("Note: The agent will only respond based on book content.")

            while True:
                try:
                    query = input("\nEnter your question: ").strip()
                    if query.lower() in ['quit', 'exit', 'q']:
                        print("Goodbye!")
                        break

                    if not query:
                        continue

                    result = agent.ask(query)

                    print(f"\nResponse: {result['response']}")
                    if result['status'] == 'insufficient_context':
                        print("(Information not available in book content)")
                    else:
                        print(f"Confidence: {result['context_evaluation']['relevance_score']:.3f}")

                except KeyboardInterrupt:
                    print("\nGoodbye!")
                    break
                except Exception as e:
                    logger.error(f"Error processing query: {str(e)}")
                    print("Sorry, I encountered an error processing your query.")

    except Exception as e:
        logger.error(f"Error initializing agent: {str(e)}")
        sys.exit(1)


if __name__ == "__main__":
    main()