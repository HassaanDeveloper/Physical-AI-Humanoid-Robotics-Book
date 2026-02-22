# Plan: FastAPI Backend and Frontend Integration

- Use the existing Docusaurus project in `book_frontend` and build a chatbot UI component inside it for full chat interaction and display Chatbot UI across the entire book frontend.
- Create a single api.py file in the project root and initialize a FastAPI server.
- In api.py, import and use the agent from agent.py to handle query processing.
- Implement a POST chat endpoint that receives user queries and returns agent responses in JSON format.
- Connect the Docusaurus chatbot UI to the FastAPI endpoint and validate end-to-end local communication.
