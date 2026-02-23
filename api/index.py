from fastapi import FastAPI
from starlette.responses import JSONResponse
import os
import sys

# Ensure backend can be imported if needed later
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

app = FastAPI()

@app.get("/")
async def read_root():
    return JSONResponse({"message": "Hello from Vercel FastAPI (Diagnostic)"})

@app.get("/test")
async def read_test():
    return JSONResponse({"message": "Test endpoint"})

@app.post("/chat") # Keep chat for testing later
async def chat_test():
    return JSONResponse({"message": "Chat endpoint (Diagnostic)"})

@app.get("/health") # Keep health for testing later
async def health_test():
    return JSONResponse({"message": "Health endpoint (Diagnostic)"})
