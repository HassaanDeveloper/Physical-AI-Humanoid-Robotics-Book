# Tasks: FastAPI Backend and Frontend Integration

This document breaks down the work into concrete tasks based on the approved plan.

## Task 1: Backend API Setup
- **1.1:** Create a new file named `api.py` in the project root directory.
- **1.2:** Add `fastapi`, `uvicorn[standard]`, and `python-dotenv` to the `dependencies` section of the main `pyproject.toml`.
- **1.3:** In `api.py`, initialize a basic FastAPI application.
- **1.4:** Configure CORS middleware in `api.py` to allow all origins, headers, and methods for local development.

## Task 2: Agent Integration
- **2.1:** In `api.py`, import the `agent_executor` from `agent.py`.
- **2.2:** Define Pydantic models for the chat request (`query: str`) and response (`response: str`).
- **2.3:** Implement a `/chat` endpoint using the `@app.post("/chat")` decorator that accepts the request model.
- **2.4:** Inside the endpoint, invoke the `agent_executor` with the user's query and return the agent's output in the response model.

## Task 3: Frontend Chatbot UI Component
- **3.1:** Create a new directory `book_frontend/src/components/Chatbot`.
- **3.2:** Inside the new directory, create `index.js` and `styles.css` for the Chatbot component.
- **3.3:** In `index.js`, build a React component with a state to manage messages, an input field, a send button, and a message display area.

## Task 4: Docusaurus Integration
- **4.1:** Use the Docusaurus "swizzle" command to get control over the root `Layout` component.
- **4.2:** Modify the swizzled `Layout` component to render the new `Chatbot` component, ensuring it appears on all pages of the site.
- **4.3:** In the `Chatbot` component, implement the `fetch` API call to the `http://127.0.0.1:8000/chat` endpoint when the send button is clicked.
- **4.4:** Update the chat's message state with the response from the backend.

## Task 5: Documentation and Verification
- **5.1:** Create a `start_dev.bat` script to concurrently run the FastAPI backend server and the Docusaurus frontend development server.
- **5.2:** Add a section to the main `README.md` explaining how to use the new script to run the integrated application and test the chat functionality.
