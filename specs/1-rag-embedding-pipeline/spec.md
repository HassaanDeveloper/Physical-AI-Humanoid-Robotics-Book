# RAG Embedding Pipeline Specification

## Feature Name
RAG Embedding Pipeline

## Short Name
rag-embedding-pipeline

## Feature Number
1

## Description
Deploy Book URLs, Generate embeddings, and Store them in a Vector Database to support RAG-based querying

## Target Audience
AI engineers and developers building RAG systems on top of static documentation websites.

## Focus Areas
- Extracting content from deployed Vercel URLs only
- Chunking and preprocessing book content
- Generating embeddings using Cohere models
- Storing vectors and metadata in Qdrant

## Success Criteria
- All book pages are successfully ingested
- Embeddings are generated and stored without loss
- Each chunk is traceable to its source URL and section
- Vector search returns relevant chunks for test queries

## Constraints
- Embeddings: Cohere embedding models
- Vector DB: Qdrant Cloud (Free Tier)
- Input source: Deployed Docusaurus website URLs
- Deterministic chunking strategy
- No hallucinated or synthetic content

## Out of Scope
- Retrieval or ranking logic
- Agent reasoning or orchestration
- Frontend or API integration
- Model fine-tuning or training

## User Scenarios

### Scenario 1: Ingest Book Content
**Actor**: AI Engineer
**Precondition**: Book website is deployed on Vercel
**Flow**:
1. Engineer provides book URLs to the pipeline
2. System fetches HTML content from each URL
3. System extracts main content and metadata
4. System chunks content using deterministic strategy
5. System generates embeddings for each chunk
6. System stores vectors and metadata in Qdrant
**Postcondition**: All book content is available for semantic search

### Scenario 2: Semantic Search
**Actor**: Developer
**Precondition**: Content is ingested and stored in Qdrant
**Flow**:
1. Developer submits a query to the search system
2. System generates embedding for the query
3. System searches Qdrant for similar vectors
4. System returns relevant chunks with metadata
5. Developer can trace chunks back to source URLs
**Postcondition**: Developer receives relevant content chunks for their query

## Functional Requirements

### Content Ingestion
- [REQ-001] System shall fetch HTML content from provided Vercel URLs
- [REQ-002] System shall extract main content from Docusaurus HTML structure
- [REQ-003] System shall preserve metadata (title, description, keywords)
- [REQ-004] System shall handle fetch errors gracefully and continue processing

### Content Chunking
- [REQ-005] System shall split content into chunks of configurable size (default: 500 tokens)
- [REQ-006] System shall use sentence-based splitting for natural chunk boundaries
- [REQ-007] System shall implement configurable overlap between chunks (default: 50 tokens)
- [REQ-008] System shall use deterministic chunking strategy for reproducibility
- [REQ-009] System shall count tokens accurately using appropriate tokenizer

### Embedding Generation
- [REQ-010] System shall generate embeddings using Cohere embed-english-v3.0 model
- [REQ-011] System shall process embeddings in batches (default: 96 items per batch)
- [REQ-012] System shall implement rate limiting between batches (default: 1.0 seconds)
- [REQ-013] System shall handle API errors and implement retry logic
- [REQ-014] System shall validate embedding dimensions match expected size (1024)

### Vector Storage
- [REQ-015] System shall store vectors in Qdrant Cloud
- [REQ-016] System shall create collection with appropriate vector configuration
- [REQ-017] System shall store metadata with each vector (URL, title, chunk info)
- [REQ-018] System shall use deterministic ID generation for traceability
- [REQ-019] System shall implement batch upsert operations (default: 100 points per batch)

### Search Functionality
- [REQ-020] System shall support semantic search using vector similarity
- [REQ-021] System shall return configurable number of results (default: 5)
- [REQ-022] System shall include metadata and similarity scores in search results
- [REQ-023] System shall support filtering by source URL or metadata

## Key Entities

### BookContent
- URL: string (source URL)
- Title: string (page title)
- Content: string (raw HTML content)
- Metadata: object (description, keywords, language)
- Timestamp: datetime (ingestion time)

### ContentChunk
- Text: string (chunk content)
- TokenCount: integer (number of tokens)
- ChunkIndex: integer (position in sequence)
- ChunkTotal: integer (total chunks from source)
- SourceURL: string (original URL)
- Title: string (original title)
- Timestamp: datetime (processing time)

### VectorRecord
- ID: string (deterministic identifier)
- Vector: float[1024] (embedding)
- Text: string (chunk content)
- URL: string (source URL)
- Title: string (source title)
- ChunkIndex: integer (chunk position)
- ChunkTotal: integer (total chunks)
- TokenCount: integer (token count)
- Timestamp: datetime (processing time)

## Assumptions

1. **Input Source**: Book content is available on deployed Vercel URLs using Docusaurus structure
2. **Content Structure**: Main content is contained in div.theme-doc-markdown elements
3. **Tokenization**: Using cl100k_base tokenizer for accurate token counting
4. **Embedding Model**: Cohere embed-english-v3.0 produces 1024-dimensional vectors
5. **Qdrant Configuration**: Free tier supports required collection size and operations
6. **Error Handling**: Network errors and API failures are transient and can be retried
7. **Performance**: Batch processing provides sufficient throughput for expected content volume

## Dependencies

1. **External Services**:
   - Vercel: Hosting for book content
   - Cohere API: Embedding generation service
   - Qdrant Cloud: Vector database storage

2. **Data Sources**:
   - Deployed book documentation URLs
   - Docusaurus HTML structure

3. **Technical Dependencies**:
   - Python 3.9+ runtime environment
   - UV package manager for dependency management
   - Network connectivity to external services

## Acceptance Criteria

### Content Ingestion
- ✅ All provided URLs are processed successfully
- ✅ Main content is extracted without navigation/boilerplate
- ✅ Metadata is preserved and associated with content
- ✅ Error handling allows partial success on network failures

### Content Chunking
- ✅ Content is split into chunks of specified size
- ✅ Chunk boundaries respect sentence structure
- ✅ Overlapping chunks maintain context continuity
- ✅ Token counting is accurate and reproducible
- ✅ Chunking strategy produces deterministic results

### Embedding Generation
- ✅ All content chunks generate valid embeddings
- ✅ Batch processing completes without API errors
- ✅ Rate limiting prevents API throttling
- ✅ Embedding dimensions match expected size
- ✅ Error recovery allows continuation after failures

### Vector Storage
- ✅ All embeddings are stored in Qdrant
- ✅ Collection is created with correct configuration
- ✅ Metadata is stored and retrievable with vectors
- ✅ Batch operations complete successfully
- ✅ Vector IDs allow traceability to source content

### Search Functionality
- ✅ Search returns relevant results for test queries
- ✅ Results include metadata and similarity scores
- ✅ Traceability allows navigation to source content
- ✅ Performance meets user expectations
- ✅ Filtering works as expected

## Test Scenarios

### Test Case 1: Basic Ingestion Pipeline
**Description**: Test complete pipeline with sample book URLs
**Steps**:
1. Provide 3-5 sample book URLs
2. Run ingestion pipeline
3. Verify all URLs processed successfully
4. Check chunk count and distribution
5. Verify embeddings generated for all chunks
6. Confirm vectors stored in Qdrant
**Expected**: All content ingested, chunked, embedded, and stored without errors

### Test Case 2: Chunking Validation
**Description**: Validate deterministic chunking behavior
**Steps**:
1. Process same content twice
2. Compare chunk boundaries and counts
3. Verify token counts match
4. Check overlap consistency
**Expected**: Identical results from both runs, demonstrating determinism

### Test Case 3: Search Relevance
**Description**: Test semantic search quality
**Steps**:
1. Ingest sample content
2. Submit relevant test queries
3. Evaluate search results for relevance
4. Check metadata completeness
5. Verify traceability to source
**Expected**: Relevant results returned with complete metadata and traceability

### Test Case 4: Error Handling
**Description**: Test pipeline resilience
**Steps**:
1. Provide mix of valid and invalid URLs
2. Run ingestion pipeline
3. Verify valid URLs processed
4. Check error handling for invalid URLs
5. Confirm pipeline completes despite errors
**Expected**: Partial success with graceful error handling

## Performance Requirements

1. **Throughput**: Process 100 pages/hour with default batch settings
2. **Latency**: Search queries return results in <500ms
3. **Reliability**: 99% success rate for valid URLs
4. **Scalability**: Support up to 10,000 content chunks
5. **Resource Usage**: Memory usage <1GB during processing

## Security Considerations

1. **API Keys**: Secure storage and handling of Cohere and Qdrant credentials
2. **Data Privacy**: No PII or sensitive content in book materials
3. **Network Security**: HTTPS for all external communications
4. **Rate Limiting**: Prevent abuse of external APIs
5. **Input Validation**: Validate URLs before processing

## Monitoring and Observability

1. **Logging**: Comprehensive logging of pipeline operations
2. **Metrics**: Track processing times, success rates, error counts
3. **Alerting**: Notify on critical failures or performance issues
4. **Traceability**: Maintain audit trail of processing activities
5. **Health Checks**: Monitor service availability and performance

## Deployment Requirements

1. **Environment**: Python 3.9+ with UV package manager
2. **Configuration**: Environment variables for API keys and settings
3. **Dependencies**: All required packages specified in pyproject.toml
4. **Infrastructure**: Network access to Vercel, Cohere, and Qdrant services
5. **Scheduling**: Ability to run on-demand or scheduled basis

## Future Enhancements

1. **Incremental Updates**: Detect and process only changed content
2. **Multi-language Support**: Handle non-English content appropriately
3. **Advanced Chunking**: Content-aware chunking strategies
4. **Performance Optimization**: Parallel processing and caching
5. **Monitoring Dashboard**: Visualization of pipeline metrics

## Glossary

- **RAG**: Retrieval-Augmented Generation - AI technique combining retrieval and generation
- **Embedding**: Vector representation of text for semantic similarity
- **Chunking**: Splitting content into manageable pieces for processing
- **Deterministic**: Producing consistent, reproducible results
- **Vector Database**: Database optimized for storing and querying vector embeddings