@echo off
echo Starting RAG Agent API server...

REM Install backend dependencies
pip install -e .

REM Start the FastAPI server
uvicorn api:app --host 0.0.0.0 --port 8000 --reload