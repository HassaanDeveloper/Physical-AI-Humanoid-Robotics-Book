import os
import sys

# Add the backend directory to sys.path
sys.path.append(os.path.join(os.path.dirname(__file__), "backend"))

# Now import the app from backend.api
try:
    from backend.api import app
except ImportError:
    from api import app

# This allows Vercel's FastAPI builder to find the 'app' object
if __name__ == "__main__":
    import uvicorn
    host = os.getenv("HOST", "0.0.0.0")
    port = int(os.getenv("PORT", 8000))
    uvicorn.run(app, host=host, port=port)
