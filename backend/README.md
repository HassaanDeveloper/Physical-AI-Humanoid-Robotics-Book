# RAG Pipeline Validation

This script connects to Qdrant to retrieve stored embeddings and validates the end-to-end retrieval pipeline to ensure correct, relevant, and traceable context is returned for queries.

## Setup

1. **Prerequisites**:
   - Python 3.8 or higher
   - Pip package manager
   - Access to Qdrant instance (either local or cloud)
   - Cohere API key for embedding generation

2. **Installation**:
   ```bash
   pip install qdrant-client cohere python-dotenv
   ```

3. **Environment Configuration**:
   Create a `.env` file in the backend directory:
   ```env
   QDRANT_URL=https://your-qdrant-instance-url
   QDRANT_API_KEY=your-qdrant-api-key
   COHER_API_KEY=your-cohere-api-key
   COLLECTION_NAME=book_content
   ```

## Running the Validation

1. **Navigate to backend directory**:
   ```bash
   cd backend
   ```

2. **Run the validation script**:
   ```bash
   python retrieve.py
   ```

3. **Or run with specific query**:
   ```bash
   python retrieve.py --query "What are VLA foundations?"
   ```

## Sample Usage

```python
from retrieve import RAGValidator

# Initialize validator
validator = RAGValidator()

# Validate retrieval pipeline
validation_result = validator.validate_pipeline()

# Run specific query
results = validator.search_similar("vision language action systems")

# Validate content accuracy
accuracy_report = validator.validate_content_accuracy("query", "retrieved content")
```

## Expected Output

The validation script will output:
- Connection status to Qdrant
- Number of vectors loaded from the collection
- Sample search results for test queries
- Content accuracy validation results
- Metadata traceability validation
- Overall validation summary

## Troubleshooting

- **Connection errors**: Verify QDRANT_URL and QDRANT_API_KEY in .env file
- **Empty results**: Ensure the collection name is correct and contains data
- **Embedding errors**: Verify COHERE_API_KEY in .env file
