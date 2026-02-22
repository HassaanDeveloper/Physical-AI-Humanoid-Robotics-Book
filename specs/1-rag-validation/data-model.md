# Data Model: RAG Pipeline Validation

## Entities

### RetrievalResult
- **fields**:
  - id: string (unique identifier for the result)
  - query: string (the original query text)
  - retrieved_content: string (the content retrieved from Qdrant)
  - source_content: string (the original content from the book for comparison)
  - similarity_score: float (cosine similarity score between query and retrieved content)
  - metadata: object (metadata associated with the retrieved content including URL, title, etc.)
  - timestamp: datetime (when the retrieval was performed)
  - validation_passed: boolean (whether content validation passed)

### ValidationResult
- **fields**:
  - id: string (unique identifier for the validation)
  - query: string (the original query text)
  - accuracy_score: float (percentage of content that matches source)
  - metadata_valid: boolean (whether metadata links correctly to source)
  - relevance_score: float (how relevant the retrieved content is to the query)
  - validation_results: array (individual validation results for each retrieved passage)
  - timestamp: datetime (when validation was performed)

### QueryTestSuite
- **fields**:
  - id: string (unique identifier for the test suite)
  - name: string (name/description of the test suite)
  - queries: array (list of test queries to validate the pipeline)
  - expected_results: array (expected results for each query for validation)
  - test_category: string (category of test, e.g., technical concepts, general knowledge)
  - created_at: datetime (when the test suite was created)

## Relationships

- A `QueryTestSuite` contains multiple test queries
- Each query generates multiple `RetrievalResult` entries
- Multiple `RetrievalResult` entries contribute to a single `ValidationResult`

## Validation Rules

- `RetrievalResult.similarity_score` must be between 0 and 1
- `RetrievalResult.validation_passed` must be a boolean value
- `ValidationResult.accuracy_score` must be between 0 and 100
- `RetrievalResult.metadata` must contain required fields (url, title, source)
- All string fields must be non-empty when required

## State Transitions

- `RetrievalResult` starts in "pending" state and moves to "completed" after processing
- `ValidationResult` moves from "processing" to "completed" after all validations are done