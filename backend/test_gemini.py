#!/usr/bin/env python3

import os
import warnings
import google.genai as genai

# Configure API key
api_key = os.getenv("GEMINI_API_KEY")
if not api_key:
    print("GEMINI_API_KEY environment variable is not set")
    exit(1)

genai.configure(api_key=api_key)

# Try to initialize a model
try:
    model = genai.GenerativeModel(model_name='gemini-1.5-flash')
    print("Model initialized successfully")

    # Try a simple test
    response = model.generate_content("Hello, world!")
    print("API call successful:", response.text[:100] + "..." if len(response.text) > 100 else response.text)
except Exception as e:
    print(f"Error: {e}")