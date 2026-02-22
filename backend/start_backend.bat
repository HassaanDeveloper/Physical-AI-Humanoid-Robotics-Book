@echo off
echo Starting RAG Agent API server...

REM Install backend dependencies
pip install -e backend/

REM Start the FastAPI server
uvicorn backend.main:app --host 0.0.0.0 --port 8000 --reload