Spec: Spec-4 – FastAPI Backend and Frontend Integration

Purpose:
Integrate the RAG agent backend with the Docusaurus frontend by building a FastAPI service that exposes chat endpoints and connects locally to the agent pipeline.

Target Audience:
AI engineers integrating RAG systems into web applications.

Focus:
- Creating FastAPI server
- Exposing chat endpoint(s)
- Connecting FastAPI to agent (Spec-3)
- Handling request/response flow between frontend and backend

Success Criteria:
- Frontend successfully sends queries to backend
- Backend calls agent and returns grounded responses
- End-to-end flow works locally
- Proper error handling and validation implemented

Constraints:
- Backend: FastAPI
- Agent logic from agent.py
- Local connection for development
- JSON-based request/response format

Not Building:
- Deployment to cloud
- Authentication or user accounts
- UI redesign
- Multi-agent orchestration
