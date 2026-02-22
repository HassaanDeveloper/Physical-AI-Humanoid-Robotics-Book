#!/usr/bin/env python3
"""
Test script to verify the FastAPI backend works correctly
"""

import sys
import os
sys.path.insert(0, os.path.dirname('.'))

print("Testing FastAPI backend import...")

try:
    # Import the app without triggering the agent initialization
    import importlib.util
    spec = importlib.util.spec_from_file_location("backend_app", "backend/main.py")
    backend_app = importlib.util.module_from_spec(spec)

    # Execute the module but don't trigger the main function
    # This will define the app but not initialize the agent yet
    spec.loader.exec_module(backend_app)

    print("✓ Backend module imported successfully")
    print("✓ App object created:", hasattr(backend_app, 'app'))
    print("✓ App title:", getattr(backend_app.app, 'title', 'No title'))

    # Check routes
    routes = [(route.path, route.methods) for route in backend_app.app.routes if hasattr(route, 'path')]
    print("✓ Defined routes:")
    for path, methods in routes:
        print(f"  - {', '.join(methods)} {path}")

except Exception as e:
    print(f"✗ Error: {e}")
    import traceback
    traceback.print_exc()

print("\nTesting completed.")