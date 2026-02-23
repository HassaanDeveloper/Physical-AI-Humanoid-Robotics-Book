import os
import sys

# Add the project root to sys.path so we can import from backend/
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Import the FastAPI app from backend/api.py
try:
    from backend.api import app
except ImportError:
    # Fallback if the pathing is slightly different on Vercel
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'backend'))
    from api import app

# This is the entrypoint for Vercel
# The 'app' object MUST be at the top level of the file
