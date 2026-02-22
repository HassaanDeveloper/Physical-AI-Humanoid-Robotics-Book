# Quickstart Guide: OpenAI Agent with Retrieval Integration

## Setup

1. **Prerequisites**:
   - Python 3.8 or higher
   - Pip package manager
   - OpenAI API key for agent creation
   - Access to Qdrant instance (from Spec-2)
   - Cohere API key for embedding generation

2. **Installation**:
   ```bash
   pip install openai qdrant-client cohere python-dotenv
   ```

3. **Environment Configuration**:
   Create a `.env` file in the project root:
   ```env
   OPENAI_API_KEY=your-openai-api-key
   QDRANT_URL=https://your-qdrant-instance-url
   QDRANT_API_KEY=your-qdrant-api-key
   COHERE_API_KEY=your-cohere-api-key
   COLLECTION_NAME=book_content
   ```

## Running the Agent

1. **Run the agent**:
   ```bash
   python agent.py
   ```

2. **Or run with specific query**:
   ```bash
   python agent.py --query "What are VLA systems?"
   ```

## Sample Usage

```python
from agent import OpenAIBookAgent

# Initialize agent
agent = OpenAIBookAgent()

# Ask a question
response = agent.ask("Explain the foundations of vision-language-action systems")

# Or interact in a conversation
result = agent.conversation([
    {"role": "user", "content": "What are VLA systems?"}
])
```

## Expected Output

The agent will:
- Call the Qdrant retrieval tool to get relevant book content
- Generate a response based only on the retrieved content
- Refuse to answer if the context is insufficient
- Never include hallucinated information

## Troubleshooting

- **API errors**: Verify OPENAI_API_KEY, QDRANT_API_KEY, and COHERE_API_KEY in .env file
- **Empty results**: Ensure the Qdrant collection contains data from Spec-1
- **Connection errors**: Verify QDRANT_URL in .env file