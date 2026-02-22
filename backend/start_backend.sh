#!/bin/bash
# Start the FastAPI backend server

echo "Starting RAG Agent API server..."

# Install backend dependencies
pip install -e backend/

# Start the FastAPI server
uvicorn backend.main:app --host 0.0.0.0 --port 8000 --reload