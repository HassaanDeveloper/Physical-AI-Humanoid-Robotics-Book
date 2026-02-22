# Implementation Plan: URL Ingestion & Embedding Pipeline

## Feature Name
Spec-1: URL Ingestion & Embedding Pipeline

## Overview
This plan outlines the implementation of **Spec-1: URL Ingestion & Embedding Pipeline** using Python and UV package manager, replacing the existing JavaScript implementation with a single `main.py` file approach.

## Current State Analysis

### Existing JavaScript Implementation
- ✅ Complete website ingestion pipeline in JavaScript
- ✅ Uses Cohere embeddings and Qdrant storage
- ✅ Modular architecture with separate files
- ✅ All dependencies installed and working

### Python Implementation Requirements
- 📋 Create `backend/` folder with Python project
- 📋 Use UV for dependency management
- 📋 Single `main.py` file containing all logic
- 📋 Fetch deployed book URLs and extract content
- 📋 Chunk content optimally for embeddings
- 📋 Generate embeddings using Cohere
- 📋 Store vectors with metadata in Qdrant
- 📋 Main function to orchestrate full pipeline

## Technical Context

### Technology Stack
- **Language**: Python 3.9+
- **Package Manager**: UV (faster alternative to pip)
- **HTTP Client**: `requests` or `httpx` (async)
- **HTML Parsing**: `beautifulsoup4`
- **Embeddings**: `cohere` Python SDK
- **Vector DB**: `qdrant-client`
- **Tokenization**: `tiktoken` (for accurate counting)
- **Configuration**: `python-dotenv`
- **Data Validation**: `pydantic`

### Key Differences from JavaScript
1. **Single File Architecture**: All logic in `main.py` vs modular JavaScript
2. **Async/Await**: Python's async I/O pattern
3. **Type Safety**: Optional static typing
4. **Token Counting**: Accurate with `tiktoken` vs estimation
5. **Dependency Management**: UV vs npm

### NEEDS CLARIFICATION
- Should the Python implementation completely replace JavaScript or coexist?
- Are there specific performance requirements for the Python version?
- Should we maintain API compatibility with existing JavaScript implementation?
- Any specific Python version requirements or constraints?
- Should error handling patterns match JavaScript implementation?

## Constitution Check

### Alignment with Project Goals
- ✅ **Modularity**: Single file approach simplifies deployment
- ✅ **Performance**: Python async can be more efficient
- ✅ **Maintainability**: Clear, linear code structure
- ✅ **Extensibility**: Easy to add new features
- ✅ **Documentation**: Comprehensive docstrings and comments

### Potential Violations
- ⚠️ **Single File vs Modular**: Violates separation of concerns
- ⚠️ **No Unit Tests**: Harder to test single large file
- ⚠️ **Limited Reusability**: Components can't be imported separately

**Justification**: User explicitly requested single `main.py` file approach for simplicity and ease of deployment.

## Implementation Plan

### Phase 1: Project Setup

**Step 1: Create Project Structure**
```bash
mkdir -p backend
cd backend
uv init
```

**Step 2: Create `pyproject.toml`**
```toml
[project]
name = "book-hackathon-backend"
version = "0.1.0"
description = "Python URL Ingestion & Embedding Pipeline"
requires-python = ">=3.9"

dependencies = [
    "requests>=2.31.0",
    "beautifulsoup4>=4.12.0",
    "cohere>=5.0.0",
    "qdrant-client>=1.7.0",
    "python-dotenv>=1.0.0",
    "tiktoken>=0.5.0",
    "pydantic>=2.0.0",
    "httpx>=0.25.0"
]
```

**Step 3: Install Dependencies**
```bash
uv pip install -r requirements.txt
```

### Phase 2: Main.py Implementation

**File**: `backend/main.py`

```python
#!/usr/bin/env python3
"""
Book Hackathon URL Ingestion & Embedding Pipeline

Single-file implementation for fetching deployed book URLs,
extracting content, generating embeddings, and storing in Qdrant.
"""

import os
import re
import asyncio
import hashlib
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass

import requests
from bs4 import BeautifulSoup
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
import tiktoken
from pydantic import BaseModel, Field, validator
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configuration
@dataclass
class Config:
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")
    QDRANT_URL: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    QDRANT_API_KEY: Optional[str] = os.getenv("QDRANT_API_KEY")
    BASE_URL: str = os.getenv("BASE_URL", "https://book-hackathon.vercel.app")
    CHUNK_SIZE: int = 500  # tokens
    OVERLAP: int = 50  # tokens
    BATCH_SIZE: int = 96  # Cohere batch size
    RATE_LIMIT: float = 1.0  # seconds between batches

config = Config()

# Data Models
class ChunkMetadata(BaseModel):
    url: str
    title: str
    chunk_index: int
    chunk_total: int
    token_count: int
    timestamp: str
    source: str = "book-hackathon"

    @validator('url')
    def validate_url(cls, v):
        if not v.startswith('http'):
            return f"{config.BASE_URL}{v}" if not v.startswith('/') else f"{config.BASE_URL}{v}"
        return v

class ContentChunk(BaseModel):
    text: str
    metadata: ChunkMetadata
    embedding: Optional[List[float]] = None

class IngestionResult(BaseModel):
    url: str
    content: str
    metadata: Dict[str, Any]
    timestamp: str

# Main Pipeline Class
class IngestionPipeline:
    def __init__(self):
        self.cohere_client = cohere.Client(config.COHERE_API_KEY)
        self.qdrant_client = QdrantClient(
            url=config.QDRANT_URL,
            api_key=config.QDRANT_API_KEY
        )
        self.collection_name = "book_content"
        self.embedding_dimension = 1024  # Cohere embed-english-v3.0
        self.encoder = tiktoken.get_encoding("cl100k_base")

    async def initialize(self):
        """Initialize Qdrant collection"""
        try:
            await self.qdrant_client.get_collection(self.collection_name)
        except:
            await self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=self.embedding_dimension,
                    distance=models.Distance.COSINE
                )
            )

# Website Ingestion Methods
    def fetch_html(self, url_path: str) -> str:
        """Fetch HTML content from URL"""
        url = url_path if url_path.startswith('http') else f"{config.BASE_URL}{url_path}"

        try:
            response = requests.get(
                url,
                headers={'User-Agent': 'Book-Hackathon-Ingestion-Bot/1.0'},
                timeout=10
            )
            response.raise_for_status()
            return response.text
        except Exception as e:
            raise Exception(f"Failed to fetch {url}: {str(e)}")

    def extract_content(self, html: str) -> str:
        """Extract main content from HTML"""
        soup = BeautifulSoup(html, 'html.parser')

        # Remove non-content elements
        for element in soup(['nav', 'footer', 'sidebar', 'div.pagination-nav',
                           'a.edit-this-page', 'div.last-updated']):
            element.decompose()

        # Get main content
        main_content = soup.find('div', class_='theme-doc-markdown') or \
                      soup.find('div', class_='markdown')

        if not main_content:
            return ""

        # Clean and normalize text
        text = main_content.get_text()
        text = re.sub(r'\s+', ' ', text)  # Normalize whitespace
        text = text.strip()

        return text

    def extract_metadata(self, html: str) -> Dict[str, Any]:
        """Extract metadata from HTML"""
        soup = BeautifulSoup(html, 'html.parser')

        return {
            'title': soup.title.string if soup.title else 'Untitled',
            'url': config.BASE_URL,
            'description': soup.find('meta', attrs={'name': 'description'})['content'] \
                      if soup.find('meta', attrs={'name': 'description'}) else '',
            'keywords': soup.find('meta', attrs={'name': 'keywords'})['content'] \
                      if soup.find('meta', attrs={'name': 'keywords'}) else '',
            'language': soup.html.get('lang', 'en') if soup.html else 'en',
            'timestamp': self.get_current_timestamp()
        }

    def ingest_url(self, url_path: str) -> IngestionResult:
        """Ingest content from single URL"""
        html = self.fetch_html(url_path)
        content = self.extract_content(html)
        metadata = self.extract_metadata(html)

        return IngestionResult(
            url=url_path,
            content=content,
            metadata=metadata,
            timestamp=self.get_current_timestamp()
        )

    async def ingest_multiple_urls(self, url_paths: List[str]) -> List[IngestionResult]:
        """Ingest content from multiple URLs"""
        results = []

        for url_path in url_paths:
            try:
                result = self.ingest_url(url_path)
                results.append(result)
            except Exception as e:
                print(f"Warning: Skipping {url_path} due to error: {str(e)}")
                continue

        return results

# Content Chunking Methods
    def estimate_token_count(self, text: str) -> int:
        """Estimate token count using tiktoken"""
        return len(self.encoder.encode(text))

    def split_into_sentences(self, text: str) -> List[str]:
        """Split text into sentences"""
        sentences = re.split(r'(?<=[.!?])\s+', text)
        return [s.strip() for s in sentences if s.strip()]

    def create_chunks(self, text: str, metadata: Dict[str, Any]) -> List[ContentChunk]:
        """Create chunks from text with metadata"""
        sentences = self.split_into_sentences(text)
        chunks = []
        current_chunk = []
        current_tokens = 0

        for sentence in sentences:
            sentence_tokens = self.estimate_token_count(sentence)

            if current_tokens + sentence_tokens > config.CHUNK_SIZE and current_chunk:
                # Save current chunk
                chunk_text = ' '.join(current_chunk)
                chunks.append(self.create_chunk_object(chunk_text, metadata, len(chunks)))

                # Start new chunk with overlap
                overlap_start = max(0, len(current_chunk) - config.OVERLAP // 2)
                current_chunk = current_chunk[overlap_start:] + [sentence]
                current_tokens = self.estimate_token_count(' '.join(current_chunk))
            else:
                # Add to current chunk
                current_chunk.append(sentence)
                current_tokens += sentence_tokens

        # Add final chunk
        if current_chunk:
            chunk_text = ' '.join(current_chunk)
            chunks.append(self.create_chunk_object(chunk_text, metadata, len(chunks)))

        # Update metadata with total count
        for i, chunk in enumerate(chunks):
            chunk.metadata.chunk_total = len(chunks)
            chunk.metadata.chunk_index = i

        return chunks

    def create_chunk_object(self, text: str, original_metadata: Dict[str, Any], chunk_index: int) -> ContentChunk:
        """Create chunk object with metadata"""
        return ContentChunk(
            text=text.strip(),
            metadata=ChunkMetadata(
                url=original_metadata['url'],
                title=original_metadata['title'],
                chunk_index=chunk_index,
                chunk_total=-1,  # Will be updated later
                token_count=self.estimate_token_count(text),
                timestamp=self.get_current_timestamp()
            )
        )

    def process_content(self, contents: List[IngestionResult]) -> List[ContentChunk]:
        """Process content through chunking pipeline"""
        all_chunks = []

        for content in contents:
            chunks = self.create_chunks(content.content, content.metadata)
            all_chunks.extend(chunks)

        return all_chunks

# Embedding Generation Methods
    async def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for single text"""
        if not text.strip():
            raise ValueError("Empty text cannot be embedded")

        response = await self.cohere_client.embed(
            texts=[text.strip()],
            model="embed-english-v3.0",
            input_type="search_document"
        )

        return response.embeddings[0]

    async def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for multiple texts (batched)"""
        valid_texts = [t for t in texts if t and t.strip()]
        all_embeddings = []

        for i in range(0, len(valid_texts), config.BATCH_SIZE):
            batch = valid_texts[i:i + config.BATCH_SIZE]

            try:
                response = await self.cohere_client.embed(
                    texts=batch,
                    model="embed-english-v3.0",
                    input_type="search_document"
                )
                all_embeddings.extend(response.embeddings)

                # Rate limiting
                if i + config.BATCH_SIZE < len(valid_texts):
                    await asyncio.sleep(config.RATE_LIMIT)

            except Exception as e:
                print(f"Warning: Failed to generate embeddings for batch {i}-{i + config.BATCH_SIZE}: {str(e)}")
                continue

        return all_embeddings

    async def generate_chunk_embeddings(self, chunks: List[ContentChunk]) -> List[ContentChunk]:
        """Generate embeddings for chunks"""
        texts = [chunk.text for chunk in chunks]
        embeddings = await self.generate_embeddings(texts)

        for i, chunk in enumerate(chunks):
            chunk.embedding = embeddings[i] if i < len(embeddings) else None

        return chunks

# Vector Storage Methods
    def generate_id(self, chunk: ContentChunk) -> str:
        """Generate deterministic ID for chunk"""
        url_hash = hashlib.sha256(chunk.metadata.url.encode()).hexdigest()[:8]
        return f"{url_hash}_{chunk.metadata.chunk_index}"

    async def store_chunks(self, chunks: List[ContentChunk]) -> List[models.UpdateResult]:
        """Store chunks in Qdrant"""
        points = []

        for chunk in chunks:
            points.append(models.PointStruct(
                id=self.generate_id(chunk),
                vector=chunk.embedding,
                payload={
                    'text': chunk.text,
                    'url': chunk.metadata.url,
                    'title': chunk.metadata.title,
                    'chunk_index': chunk.metadata.chunk_index,
                    'chunk_total': chunk.metadata.chunk_total,
                    'token_count': chunk.metadata.token_count,
                    'timestamp': chunk.metadata.timestamp,
                    'source': 'book-hackathon'
                }
            ))

        # Process in batches
        results = []
        for i in range(0, len(points), 100):
            batch = points[i:i + 100]

            try:
                result = await self.qdrant_client.upsert(
                    collection_name=self.collection_name,
                    points=batch,
                    wait=True
                )
                results.append(result)
            except Exception as e:
                print(f"Error storing batch {i}-{i + 100}: {str(e)}")
                raise

        return results

    async def search(self, query: str, limit: int = 5) -> List[models.ScoredPoint]:
        """Search for similar vectors"""
        query_embedding = await self.generate_embedding(query)

        return await self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit,
            with_payload=True,
            with_vectors=False
        )

# Utility Methods
    def get_current_timestamp(self) -> str:
        """Get current timestamp in ISO format"""
        from datetime import datetime
        return datetime.now().isoformat()

    async def get_statistics(self) -> Dict[str, Any]:
        """Get pipeline statistics"""
        try:
            collection_info = await self.qdrant_client.get_collection(self.collection_name)
            return {
                'points_count': collection_info.points_count,
                'embedding_dimension': self.embedding_dimension,
                'chunk_size': config.CHUNK_SIZE,
                'overlap': config.OVERLAP
            }
        except Exception as e:
            return {'error': str(e)}

# Main Pipeline Orchestration
    async def run_pipeline(self, url_paths: List[str]) -> Dict[str, Any]:
        """Run complete ingestion pipeline"""
        results = {
            'ingestion': [],
            'chunking': [],
            'embeddings': [],
            'storage': []
        }

        try:
            # Step 1: Website Ingestion
            print("Starting website ingestion...")
            results['ingestion'] = await self.ingest_multiple_urls(url_paths)
            print(f"Ingested {len(results['ingestion'])} pages successfully")

            # Step 2: Content Chunking
            print("Starting content chunking...")
            results['chunking'] = self.process_content(results['ingestion'])
            print(f"Created {len(results['chunking'])} chunks")

            # Step 3: Embedding Generation
            print("Starting embedding generation...")
            results['embeddings'] = await self.generate_chunk_embeddings(results['chunking'])
            print(f"Generated embeddings for {len(results['embeddings'])} chunks")

            # Step 4: Vector Storage
            print("Starting vector storage...")
            results['storage'] = await self.store_chunks(results['embeddings'])
            print(f"Stored {len(results['embeddings'])} chunks in Qdrant")

            return results

        except Exception as e:
            print(f"Pipeline failed: {str(e)}")
            raise

# Main Function
async def main():
    """Main function to orchestrate the full ingestion pipeline"""
    print("=== Book Hackathon Python Ingestion Pipeline ===\n")

    # Validate configuration
    if not config.COHERE_API_KEY:
        raise ValueError("COHERE_API_KEY environment variable is required")

    # Initialize pipeline
    pipeline = IngestionPipeline()
    await pipeline.initialize()

    # Default URLs to ingest (Module 4 content)
    default_urls = [
        '/docs/module4/vla-foundations',
        '/docs/module4/voice-to-action',
        '/docs/module4/llm-cognitive-planning',
        '/docs/module4/autonomous-humanoid-capstone'
    ]

    # Run pipeline
    print("Running ingestion pipeline...")
    results = await pipeline.run_pipeline(default_urls)

    # Display results
    print("\n=== Pipeline Results ===")
    print(f"Ingested pages: {len(results['ingestion'])}")
    print(f"Created chunks: {len(results['chunking'])}")
    print(f"Generated embeddings: {len(results['embeddings'])}")
    print(f"Stored in Qdrant: {len(results['embeddings'])} chunks")

    # Test search functionality
    print("\n=== Testing Search ===")
    query = "What are the foundations of vision-language-action systems?"
    search_results = await pipeline.search(query, 3)

    print(f"Search results for '{query}':")
    for i, result in enumerate(search_results):
        payload = result.payload
        print(f"\nResult {i+1} (Score: {result.score:.4f}):")
        print(f"Title: {payload['title']}")
        print(f"URL: {payload['url']}")
        print(f"Text: {payload['text'][:150]}...")

    # Display statistics
    stats = await pipeline.get_statistics()
    print(f"\n=== Statistics ===")
    print(f"Points in collection: {stats.get('points_count', 'N/A')}")
    print(f"Embedding dimension: {stats.get('embedding_dimension', 'N/A')}")
    print(f"Chunk size: {stats.get('chunk_size', 'N/A')} tokens")

    print("\n=== Pipeline Completed Successfully ===")

if __name__ == "__main__":
    asyncio.run(main())
```

### Phase 3: Testing and Verification

**Step 1: Create Test Script**
```bash
# backend/test_pipeline.py
test_urls = [
    '/docs/module4/vla-foundations',
    '/docs/module4/voice-to-action'
]

# Run test
python -m pytest test_pipeline.py -v
```

**Step 2: Environment Setup**
```bash
# Create .env file
echo "COHERE_API_KEY=your_key_here" > backend/.env
echo "QDRANT_URL=http://localhost:6333" >> backend/.env
```

**Step 3: Run Pipeline**
```bash
cd backend
python main.py
```

### Phase 4: Deployment

**Step 1: Build Package**
```bash
uv pip install build
python -m build
```

**Step 2: Create Dockerfile**
```dockerfile
FROM python:3.9-slim

WORKDIR /app
COPY . .

RUN uv pip install --system -r requirements.txt

CMD ["python", "main.py"]
```

**Step 3: Build and Run**
```bash
docker build -t book-hackathon-backend .
docker run -it --env-file .env book-hackathon-backend
```

## Verification Plan

### End-to-End Testing
1. **Environment Setup**: Verify UV installs all dependencies correctly
2. **Configuration**: Verify .env variables are loaded properly
3. **Website Ingestion**: Test with actual book URLs
4. **Content Chunking**: Verify chunk sizes and metadata preservation
5. **Embedding Generation**: Verify Cohere API integration
6. **Vector Storage**: Verify Qdrant storage and retrieval
7. **Search Functionality**: Test semantic search with sample queries
8. **Error Handling**: Test with invalid URLs and edge cases

### Success Criteria
- ✅ All dependencies install without errors
- ✅ Pipeline runs to completion without crashes
- ✅ Content is successfully ingested from all URLs
- ✅ Chunks are created with proper metadata
- ✅ Embeddings are generated and stored in Qdrant
- ✅ Search returns relevant results
- ✅ Statistics are accurate and up-to-date

### Test Commands
```bash
# Test dependency installation
uv pip install -r requirements.txt

# Run pipeline
python main.py

# Test with specific URLs
python -c "
import asyncio
from main import IngestionPipeline, Config

async def test():
    pipeline = IngestionPipeline()
    await pipeline.initialize()
    results = await pipeline.run_pipeline(['/docs/module4/vla-foundations'])
    print(f'Success: {len(results["embeddings"])} chunks processed')

asyncio.run(test())
"

# Test search
python -c "
import asyncio
from main import IngestionPipeline

async def test_search():
    pipeline = IngestionPipeline()
    await pipeline.initialize()
    results = await pipeline.search('What is VLA?', 3)
    print(f'Found {len(results)} results')

asyncio.run(test_search())
"
```

## Critical Files to Modify

1. **`backend/main.py`** - Complete pipeline implementation
2. **`backend/pyproject.toml`** - Project configuration
3. **`backend/requirements.txt`** - Dependency specification
4. **`backend/.env`** - Environment variables
5. **`backend/Dockerfile`** - Container configuration

## Implementation Timeline

1. **Day 1**: Project setup and dependency management
2. **Day 2**: Website ingestion and content chunking
3. **Day 3**: Embedding generation and vector storage
4. **Day 4**: Main pipeline orchestration and testing
5. **Day 5**: Documentation and deployment

## Risks and Mitigations

### Risks
1. **UV Compatibility**: New package manager may have issues
2. **Async Complexity**: Python async/await can be tricky
3. **Token Counting**: Different from JavaScript estimation
4. **Qdrant Compatibility**: Python client differences
5. **Error Handling**: Single file makes it harder

### Mitigations
1. **Fallback to pip**: If UV has issues
2. **Comprehensive Testing**: Test async functions thoroughly
3. **Accurate Tokenization**: Use tiktoken for precision
4. **Client Validation**: Test Qdrant operations early
5. **Structured Exceptions**: Custom exception classes

## Conclusion

This plan outlines a complete Python implementation of the URL ingestion and embedding pipeline using UV package manager. The single-file approach simplifies deployment while maintaining all functionality of the JavaScript version. The implementation leverages Python's strengths in async programming, type safety, and data processing while using modern dependency management with UV.