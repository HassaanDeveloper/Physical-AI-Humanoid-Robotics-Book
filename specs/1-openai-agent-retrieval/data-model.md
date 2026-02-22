# Data Model: OpenAI Agent with Retrieval Integration

## Entities

### OpenAIAgent
- **fields**:
  - id: string (unique identifier for the agent)
  - name: string (name of the agent)
  - configuration: object (OpenAI agent configuration parameters)
  - tools: array (list of integrated tools including Qdrant retrieval)
  - created_at: datetime (when the agent was created)
  - updated_at: datetime (when the agent was last updated)

### RetrievalTool
- **fields**:
  - id: string (unique identifier for the tool)
  - name: string (name of the retrieval tool)
  - description: string (description of what the tool does)
  - parameters: object (configuration for the Qdrant retrieval)
  - qdrant_config: object (Qdrant connection and collection settings)
  - embedding_model: string (embedding model used for queries)
  - max_results: integer (maximum number of results to retrieve)

### RetrievedChunks
- **fields**:
  - query: string (the original query sent to the retrieval tool)
  - chunks: array (list of retrieved content chunks with metadata)
  - scores: array (relevance scores for each chunk)
  - timestamp: datetime (when the retrieval happened)
  - total_chunks: integer (total number of chunks retrieved)

### GroundedResponse
- **fields**:
  - id: string (unique identifier for the response)
  - query: string (original user query)
  - retrieved_context: object (the chunks used to generate the response)
  - response: string (the agent's response)
  - grounding_confidence: float (confidence that response is grounded in context)
  - hallucination_check: boolean (whether response passed hallucination checks)
  - timestamp: datetime (when the response was generated)

### ContextEvaluator
- **fields**:
  - id: string (unique identifier for the evaluation)
  - query: string (the original query)
  - retrieved_content: string (content retrieved from Qdrant)
  - sufficiency_score: float (score indicating if context is sufficient to answer)
  - adequate_for_response: boolean (whether context is sufficient)
  - evaluation_reasoning: string (reasoning behind the sufficiency assessment)

## Relationships

- An `OpenAIAgent` integrates with one or more `RetrievalTool` instances
- A `RetrievalTool` generates multiple `RetrievedChunks` per query
- `RetrievedChunks` are used to create a `GroundedResponse`
- A `ContextEvaluator` evaluates the sufficiency of `RetrievedChunks` for response generation

## Validation Rules

- `GroundedResponse.grounding_confidence` must be between 0 and 1
- `GroundedResponse.hallucination_check` must be a boolean value
- `RetrievedChunks.total_chunks` must be non-negative
- `ContextEvaluator.sufficiency_score` must be between 0 and 1
- `RetrievedChunks.chunks` must contain non-empty content when present

## State Transitions

- `RetrievedChunks` starts in "pending" state and moves to "completed" after retrieval
- `GroundedResponse` moves from "generating" to "completed" after response generation and validation
- `ContextEvaluator` moves from "evaluating" to "completed" after sufficiency assessment