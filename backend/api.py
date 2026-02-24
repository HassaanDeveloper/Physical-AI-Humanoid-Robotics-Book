import os
import sys
import logging
from typing import Dict, Any, List

# Add the current directory to sys.path to resolve local imports (e.g., agent.py)
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

from agent import GeminiBookAgent

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Pydantic models for request/response
class ChatRequest(BaseModel):
    """Request model for chat endpoint"""
    query: str
    max_chunks: int = 5


class ChatResponse(BaseModel):
    """Response model for chat endpoint"""
    query: str
    response: str
    status: str
    retrieved_chunks: List[Dict[str, Any]]
    context_evaluation: Dict[str, Any]
    timestamp: str


class HealthResponse(BaseModel):
    """Response model for health endpoint"""
    status: str
    message: str


# Create FastAPI app
app = FastAPI(
    title="RAG Agent API",
    description="FastAPI server for RAG agent integration",
    version="1.0.0",
    root_path="/api"
)

@app.get("/")
async def root_endpoint():
    """Diagnostic root endpoint"""
    return {
        "status": "online",
        "message": "RAG Agent API is running",
        "version": "1.0.0"
    }

# Add CORS middleware to allow frontend communication
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize agent (simple singleton for dev)
agent = None


@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint"""
    return HealthResponse(
        status="healthy",
        message="RAG Agent API is running"
    )


@app.get("/models")
async def get_models():
    """Returns information about the AI model being used"""
    global agent
    if agent is None:
        try:
            agent = GeminiBookAgent()
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Agent initialization failed: {str(e)}")
    
    return {
        "model_name": agent.model_name,
        "status": "active"
    }


@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Chat endpoint that processes queries using the RAG agent.

    Args:
        request: Chat request containing the query

    Returns:
        ChatResponse with the agent's response and metadata
    """
    global agent

    # Lazy initialize agent if not yet initialized
    if agent is None:
        try:
            agent = GeminiBookAgent()
            logger.info("Agent initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize agent: {e}")
            raise HTTPException(status_code=500, detail=f"Agent initialization failed: {str(e)}")

    try:
        logger.info(f"Processing chat request: {request.query}")

        # Process the query using the agent
        result = agent.ask(request.query, max_chunks=request.max_chunks)

        # Create response
        response = ChatResponse(
            query=result['query'],
            response=result['response'],
            status=result['status'],
            retrieved_chunks=result['retrieved_chunks'],
            context_evaluation=result['context_evaluation'],
            timestamp=result['timestamp']
        )

        logger.info(f"Chat response generated successfully for query: {request.query[:50]}...")
        return response

    except Exception as e:
        logger.error(f"Error processing chat request: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing request: {str(e)}")


@app.post("/query_raw")
async def query_raw_endpoint(request: ChatRequest):
    """
    Raw query endpoint for debugging.
    Returns the full agent result without response model validation.
    """
    global agent
    if agent is None:
        try:
            agent = GeminiBookAgent()
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Agent initialization failed: {str(e)}")

    try:
        result = agent.ask(request.query, max_chunks=request.max_chunks)
        return result
    except Exception as e:
        logger.error(f"Error in raw query: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


if __name__ == "__main__":
    # Get host and port from environment or use defaults
    host = os.getenv("HOST", "0.0.0.0")
    port = int(os.getenv("PORT", 8000))

    logger.info(f"Starting server on {host}:{port}")

    # Run the server
    uvicorn.run(
        "api:app",
        host=host,
        port=port,
        reload=True,  # Enable auto-reload during development
        log_level="info"
    )

