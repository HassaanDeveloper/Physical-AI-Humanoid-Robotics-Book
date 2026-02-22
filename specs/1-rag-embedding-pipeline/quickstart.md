# Quickstart Guide: URL Ingestion & Embedding Pipeline

## Prerequisites

### System Requirements
- Python 3.9 or higher
- pip or UV package manager
- Network access to Vercel, Cohere, and Qdrant services
- Minimum 2GB RAM (4GB recommended)
- 1GB free disk space

### Required Accounts
1. **Cohere API Key** - Sign up at [Cohere](https://cohere.com/)
2. **Qdrant Cloud** - Free tier available at [Qdrant Cloud](https://qdrant.to/)
3. **Vercel Deployment** - Book content deployed on Vercel

### Environment Setup

```bash
# Install Python 3.9+
# Recommended: Use pyenv or system package manager

# Verify Python installation
python --version  # Should show 3.9+

# Install UV (optional but recommended)
pip install uv
```

## Installation

### Option 1: Using UV (Recommended)

```bash
# Clone repository
git clone https://github.com/your-repo/book-hackathon.git
cd book-hackathon

# Navigate to backend directory
cd backend

# Install dependencies using UV
uv pip install -r requirements.txt
```

### Option 2: Using pip

```bash
# Clone repository
git clone https://github.com/your-repo/book-hackathon.git
cd book-hackathon

# Navigate to backend directory
cd backend

# Install dependencies using pip
pip install -r requirements.txt
```

### Environment Configuration

```bash
# Copy .env template
cp .env.example .env

# Edit .env file
nano .env

# Add your API keys
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=https://your-qdrant-instance.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here  # If required
BASE_URL=https://book-hackathon.vercel.app
```

## Basic Usage

### Running the Pipeline

```bash
# Run the complete ingestion pipeline
python main.py
```

### Testing with Sample URLs

```bash
# Test with specific URLs
python -c "
import asyncio
from main import IngestionPipeline

async def test():
    pipeline = IngestionPipeline()
    await pipeline.initialize()

    # Test with sample URLs
    urls = [
        '/docs/module4/vla-foundations',
        '/docs/module4/voice-to-action'
    ]

    results = await pipeline.run_pipeline(urls)
    print(f'Processed {len(results[\"ingestion\"])} pages')
    print(f'Created {len(results[\"chunking\"])} chunks')
    print(f'Generated {len(results[\"embeddings\"])} embeddings')

asyncio.run(test())
"
```

### Testing Search Functionality

```bash
# Test semantic search
python -c "
import asyncio
from main import IngestionPipeline

async def test_search():
    pipeline = IngestionPipeline()
    await pipeline.initialize()

    # Run pipeline first (if not already done)
    await pipeline.run_pipeline(['/docs/module4/vla-foundations'])

    # Test search
    query = 'What are vision-language-action systems?'
    results = await pipeline.search(query, 3)

    print(f'Search results for: \"{query}\"')
    for i, result in enumerate(results):
        print(f'\nResult {i+1} (Score: {result.score:.4f}):')
        print(f'URL: {result.payload[\"url\"]}')
        print(f'Text: {result.payload[\"text\"][:100]}...')

asyncio.run(test_search())
"
```

## Configuration Options

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `COHERE_API_KEY` | (required) | Cohere API key for embeddings |
| `QDRANT_URL` | `http://localhost:6333` | Qdrant server URL |
| `QDRANT_API_KEY` | (optional) | Qdrant API key if required |
| `BASE_URL` | `https://book-hackathon.vercel.app` | Base URL for book content |
| `CHUNK_SIZE` | `500` | Target chunk size in tokens |
| `OVERLAP` | `50` | Overlap between chunks in tokens |
| `BATCH_SIZE` | `96` | Cohere API batch size |
| `RATE_LIMIT` | `1.0` | Delay between batches in seconds |

### Custom Configuration

```python
# Create custom config in your code
from main import Config

config = Config(
    COHERE_API_KEY='your_key',
    QDRANT_URL='https://your-qdrant-instance.qdrant.io',
    CHUNK_SIZE=400,  # Custom chunk size
    OVERLAP=40,      # Custom overlap
    BATCH_SIZE=64    # Smaller batch size
)
```

## Advanced Usage

### Custom URL Processing

```python
# Process custom URLs
import asyncio
from main import IngestionPipeline

async def process_custom_urls():
    pipeline = IngestionPipeline()
    await pipeline.initialize()

    # Custom URLs to process
    custom_urls = [
        '/docs/module1/introduction',
        '/docs/module2/advanced-topics',
        '/docs/module3/case-studies'
    ]

    results = await pipeline.run_pipeline(custom_urls)

    # Process results
    for result in results['ingestion']:
        print(f'Processed: {result.url}')
        print(f'Content length: {len(result.content)} characters')

asyncio.run(process_custom_urls())
```

### Batch Processing with Progress

```python
# Process URLs with progress tracking
import asyncio
from main import IngestionPipeline

async def process_with_progress():
    pipeline = IngestionPipeline()
    await pipeline.initialize()

    # Large set of URLs
    all_urls = [
        f'/docs/module{i}/section{j}'
        for i in range(1, 5)
        for j in range(1, 11)
    ]

    # Process in batches
    batch_size = 5
    for i in range(0, len(all_urls), batch_size):
        batch = all_urls[i:i + batch_size]
        print(f'Processing batch {i//batch_size + 1}/{len(all_urls)//batch_size + 1}')

        results = await pipeline.run_pipeline(batch)
        print(f'  Ingested: {len(results["ingestion"])} pages')
        print(f'  Chunks: {len(results["chunking"])}')
        print(f'  Embeddings: {len(results["embeddings"])}')

asyncio.run(process_with_progress())
```

### Custom Search with Filters

```python
# Advanced search with filters
import asyncio
from main import IngestionPipeline

async def advanced_search():
    pipeline = IngestionPipeline()
    await pipeline.initialize()

    # Ensure content is ingested first
    await pipeline.run_pipeline(['/docs/module4/vla-foundations'])

    # Search with specific query
    query = 'autonomous systems AND cognitive planning'
    results = await pipeline.search(query, 5)

    # Filter and process results
    relevant_results = []
    for result in results:
        if result.score > 0.7:  # High relevance threshold
            payload = result.payload
            if 'autonomous' in payload['text'].lower():
                relevant_results.append({
                    'score': result.score,
                    'text': payload['text'][:200],
                    'url': payload['url'],
                    'chunk': f'{payload["chunk_index"] + 1}/{payload["chunk_total"]}'
                })

    # Display filtered results
    print(f'Found {len(relevant_results)} highly relevant results:')
    for result in relevant_results:
        print(f'\nScore: {result["score"]:.3f}')
        print(f'Chunk: {result["chunk"]}')
        print(f'URL: {result["url"]}')
        print(f'Text: {result["text"]}...')

asyncio.run(advanced_search())
```

## Troubleshooting

### Common Issues

#### Missing API Keys

**Error**: `ValueError: COHERE_API_KEY environment variable is required`

**Solution**:
```bash
# Set API key in .env file
echo "COHERE_API_KEY=your_api_key_here" >> .env
```

#### Network Connectivity

**Error**: `Failed to fetch URL: ConnectionError`

**Solutions**:
1. Check internet connection
2. Verify URL is accessible
3. Check firewall/proxy settings
4. Test with different URLs

#### Qdrant Connection

**Error**: `Qdrant connection failed`

**Solutions**:
1. Verify Qdrant URL in .env
2. Check Qdrant service status
3. Test connection with Qdrant client
4. Verify API key if required

#### Cohere API Errors

**Error**: `Cohere API error: rate_limit_exceeded`

**Solutions**:
1. Increase RATE_LIMIT in .env
2. Reduce BATCH_SIZE
3. Check Cohere dashboard for usage
4. Wait and retry

### Debugging

```bash
# Run with debug logging
python -c "
import logging
logging.basicConfig(level=logging.DEBUG)
import asyncio
from main import IngestionPipeline

async def debug_run():
    pipeline = IngestionPipeline()
    await pipeline.initialize()
    results = await pipeline.run_pipeline(['/docs/module4/vla-foundations'])
    print('Debug run completed')

asyncio.run(debug_run())
"
```

## Deployment

### Docker Deployment

```bash
# Build Docker image
docker build -t book-hackathon-backend .

# Run container
docker run -it \
  --env-file .env \
  -p 8000:8000 \
  book-hackathon-backend
```

### Docker Compose

```yaml
# docker-compose.yml
version: '3.8'

services:
  backend:
    build: .
    env_file: .env
    ports:
      - "8000:8000"
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8000/health"]
      interval: 30s
      timeout: 10s
      retries: 3
```

```bash
# Start services
docker-compose up -d

# View logs
docker-compose logs -f

# Stop services
docker-compose down
```

### Production Considerations

1. **Environment Variables**: Use secret management for API keys
2. **Logging**: Configure proper log rotation
3. **Monitoring**: Set up health checks and alerts
4. **Scaling**: Consider horizontal scaling for large datasets
5. **Backups**: Regular Qdrant collection backups
6. **Updates**: Schedule regular dependency updates

## Monitoring and Maintenance

### Health Checks

```python
# Add health check endpoint
from fastapi import FastAPI
from main import IngestionPipeline

app = FastAPI()

@app.get("/health")
async def health_check():
    try:
        pipeline = IngestionPipeline()
        await pipeline.initialize()
        stats = await pipeline.get_statistics()
        return {
            "status": "healthy",
            "points_count": stats.get('points_count', 0),
            "last_updated": stats.get('last_updated', 'N/A')
        }
    except Exception as e:
        return {"status": "unhealthy", "error": str(e)}
```

### Logging Configuration

```python
# Configure logging in main.py
import logging
from logging.handlers import RotatingFileHandler

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        RotatingFileHandler('pipeline.log', maxBytes=1024*1024, backupCount=5),
        logging.StreamHandler()
    ]
)

logger = logging.getLogger('ingestion_pipeline')
```

### Performance Monitoring

```python
# Add performance monitoring
import time
from functools import wraps

def monitor_performance(func):
    @wraps(func)
    async def wrapper(*args, **kwargs):
        start_time = time.time()
        result = await func(*args, **kwargs)
        end_time = time.time()

        logger.info(f"{func.__name__} executed in {end_time - start_time:.2f} seconds")
        return result
    return wrapper

# Apply to key functions
class IngestionPipeline:
    @monitor_performance
    async def run_pipeline(self, url_paths):
        # ... existing implementation
```

## Best Practices

### Configuration Management
- Use environment variables for sensitive data
- Provide sensible defaults for non-sensitive settings
- Validate configuration at startup
- Document all configuration options

### Error Handling
- Handle network errors gracefully
- Implement retry logic for transient failures
- Provide meaningful error messages
- Log errors with context

### Performance Optimization
- Use batch processing where possible
- Implement rate limiting for APIs
- Optimize memory usage
- Monitor and tune performance

### Security
- Keep API keys secure
- Validate all inputs
- Use HTTPS for all communications
- Regularly update dependencies

## Support

### Getting Help

1. **Documentation**: Check the official documentation
2. **Community**: Join the project Slack/Discord
3. **Issues**: Report bugs on GitHub
4. **Contact**: Email support@book-hackathon.com

### Common Questions

**Q: How do I add more URLs?**
A: Add them to the URL list in main.py or pass them to run_pipeline()

**Q: Can I change chunk size?**
A: Yes, set CHUNK_SIZE in .env or modify the Config object

**Q: How do I update existing content?**
A: Re-run the pipeline - it will update existing vectors

**Q: Can I use different embedding models?**
A: Yes, modify the model parameter in generate_embedding()

## Conclusion

This quickstart guide provides everything you need to get started with the URL Ingestion & Embedding Pipeline. From basic setup to advanced usage, you can now ingest book content, generate embeddings, and perform semantic search with ease.

**Next Steps**:
1. Set up your environment
2. Configure API keys
3. Run the pipeline
4. Test search functionality
5. Integrate with your application