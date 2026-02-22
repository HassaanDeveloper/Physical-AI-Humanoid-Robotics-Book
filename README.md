# Physical AI & Humanoid Robotics Book Chatbot and RAG Pipeline

This repository hosts a comprehensive system for providing an AI chatbot interface to the "Physical AI & Humanoid Robotics" book content. It includes a robust Retrieval-Augmented Generation (RAG) pipeline for content ingestion and validation, a FastAPI backend, and a Docusaurus-based frontend.

## 🚀 Features

### 🗣️ AI Chatbot Interface
*   **Interactive Q&A:** Engage in natural language conversations with an AI assistant about the book's content.
*   **Grounded Responses:** Responses are strictly grounded in the book's material to minimize hallucination.
*   **Contextual Understanding:** Utilizes a RAG pipeline to retrieve and synthesize relevant information.

### 📈 RAG Pipeline Validation
The RAG (Retrieval-Augmented Generation) pipeline is designed for high accuracy and traceability.
*   **Qdrant Integration:** Connects to Qdrant for efficient vector storage and retrieval of content embeddings.
*   **Content Accuracy:** Validates that retrieved content matches query relevance.
*   **Metadata Traceability:** Ensures metadata correctly links to original book pages.
*   **Comprehensive Validation:** Provides detailed reports on pipeline status and performance.

### 📚 Data Ingestion Pipeline
This module handles the process of ingesting the book's content, generating embeddings, and storing them in a vector database.
*   **Website Ingestion:** Fetches and extracts content directly from the deployed Docusaurus book URLs.
*   **Content Chunking:** Splits content into optimal, context-preserving chunks for embedding.
*   **Embedding Generation:** Uses Cohere's `embed-english-v3.0` model for high-quality semantic embeddings.
*   **Vector Storage:** Stores embeddings in Qdrant with associated metadata for efficient semantic search.
*   **Deterministic Processing:** Ensures reproducible chunking and ID generation for consistent results.

## 📁 Project Structure

```
.
├── .claude/                # Configuration for Claude agent (if used)
├── .specify/               # Configuration for Specify tool (if used)
├── .venv/                  # Python virtual environment
├── backend/                # FastAPI backend application (Python)
│   ├── .env.example        # Example environment variables for backend
│   ├── main.py             # FastAPI application entry point
│   ├── agent.py            # Gemini RAG agent logic
│   ├── retrieve.py         # RAG pipeline validation script
│   ├── api.py              # Backend API utilities (if any)
│   ├── pyproject.toml      # Python project configuration (poetry)
│   ├── requirements.txt    # Python dependencies
│   ├── start_backend.sh    # Script to start backend (Linux/macOS)
│   ├── start_backend.bat   # Script to start backend (Windows)
│   └── ...                 # Other backend files (tests, etc.)
├── book_frontend/          # Docusaurus frontend application (React/JS)
│   ├── docusaurus.config.js # Docusaurus configuration
│   ├── package.json        # Frontend dependencies
│   ├── src/                # Frontend source code (components, pages)
│   ├── static/             # Static assets
│   └── ...                 # Other Docusaurus files
├── history/                # Project history or ADRs
├── isaac_ws/               # Isaac ROS workspace (if applicable)
├── node_modules/           # Root-level Node.js dependencies (if any)
├── public/                 # Root-level public assets (if any)
├── specs/                  # Project specifications and design documents
├── tools/                  # JavaScript ingestion and utility scripts
│   ├── package.json        # Dependencies for utility scripts
│   ├── package-lock.json   # Lock file for utility scripts
│   └── ...                 # Ingestion/debug/test JS scripts
└── README.md               # This README file
```

## 🛠️ Setup & Installation

### Prerequisites

Before you begin, ensure you have the following installed:

*   **Python 3.9+** (preferably managed with `pyenv` or similar)
*   **Node.js v20+** (LTS recommended)
*   **npm** (comes with Node.js)
*   **Docker Desktop** (for running Qdrant locally)
*   **Cohere API Key**: Sign up at [Cohere](https://cohere.com/)
*   **Gemini API Key**: Obtain from [Google AI Studio](https://aistudio.google.com/) or Google Cloud.

### 1. Start Qdrant (Vector Database)

The project uses Qdrant as its vector database. You can run it easily with Docker:

```bash
docker run -p 6333:6333 qdrant/qdrant
```

### 2. Backend Setup

Navigate to the `backend/` directory:

```bash
cd backend
```

Create a virtual environment and install Python dependencies:

```bash
python -m venv .venv
# On Windows:
.\.venv\Scripts\activate
# On Linux/macOS:
source .venv/bin/activate

pip install -r requirements.txt
# If using poetry for pyproject.toml, you might use:
# poetry install
```

### 3. Frontend Setup

Navigate to the `book_frontend/` directory:

```bash
cd book_frontend
```

Install Node.js dependencies:

```bash
npm install
# or yarn install
```

### 4. Environment Variables (`.env` files)

Create a `.env` file in the **`backend/` directory** and another one in the **`tools/` directory** (if you plan to run the ingestion scripts directly from there).

#### `backend/.env` (for the FastAPI application and RAG agent)

```env
# Cohere API Key (for query embeddings with Qdrant)
COHERE_API_KEY=your_cohere_api_key_here

# Gemini API Key (for LLM interactions)
GEMINI_API_KEY=your_gemini_api_key_here

# Qdrant Configuration
QDRANT_URL=http://localhost:6333 # Or your cloud Qdrant URL
QDRANT_API_KEY=your_qdrant_api_key_here # Only if using cloud Qdrant with API key

# Name of the Qdrant collection to use
COLLECTION_NAME=book_content

# FastAPI Server Host and Port (optional, defaults to 0.0.0.0:8000)
HOST=0.0.0.0
PORT=8000
```

#### `tools/.env` (for ingestion scripts, if running them manually)

```env
# Cohere API Key (for document embeddings)
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Configuration
QDRANT_URL=http://localhost:6333 # Or your cloud Qdrant URL
QDRANT_API_KEY=your_qdrant_api_key_here # Only if using cloud Qdrant with API key

# Name of the Qdrant collection to create/use
COLLECTION_NAME=book_content

# Base URL for the deployed Docusaurus book to ingest
BASE_URL=https://physical-ai-humanoid-robotics-book-seven-drab.vercel.app
# Sitemap URL for comprehensive ingestion
SITEMAP_URL=https://physical-ai-humanoid-robotics-book-seven-drab.vercel.app/sitemap.xml
```

## 🚀 Running the Application

### 1. Start the Backend Server

Navigate to the `backend/` directory and run the appropriate script:

*   **On Windows:**
    ```bash
    .\start_backend.bat
    ```
*   **On Linux/macOS:**
    ```bash
    bash start_backend.sh
    ```
    Alternatively, you can run directly using `uvicorn` (ensure your virtual environment is active):
    ```bash
    uvicorn main:app --reload --host 0.0.0.0 --port 8000
    ```
    The backend API will be available at `http://localhost:8000` (or the port specified in your `.env`).

### 2. Start the Frontend Application

Navigate to the `book_frontend/` directory:

```bash
cd book_frontend
npm start
# or yarn start
```

The Docusaurus frontend will typically open in your browser at `http://localhost:3000`.

### 3. Run the Ingestion Pipeline (if collection is empty)

If your Qdrant collection is empty, you need to run the ingestion pipeline to populate it with book content.

Navigate to the `tools/` directory:

```bash
cd tools
```

Install Node.js dependencies for the tools:

```bash
npm install
# or yarn install
```

Now, run the ingestion script (you might need to execute a specific script like `final-ingestion.js` or `run-full-ingestion.js` if they were included in `tools/`):

```bash
# Example if using a specific ingestion script:
# node final-ingestion.js
# Or if a main pipeline script exists in tools:
# node pipeline.js
```
*Note*: The original ingestion scripts were deleted as "unuseful". You might need to reconstruct or re-add a dedicated ingestion script to the `tools/` directory based on the `INGESTION_PIPELINE_README.md` details if you need to run ingestion. For this exercise, we assume Qdrant is already populated or will be populated by a separate process.

## 🧪 RAG Pipeline Validation Usage

The `backend/retrieve.py` script provides validation capabilities for the RAG pipeline.

Navigate to the `backend/` directory and activate your virtual environment:

```bash
cd backend
# On Windows:
.\.venv\Scripts\activate
# On Linux/macOS:
source .venv/bin/activate
```

### 1. Run Full Pipeline Validation (Default)

Validate the entire RAG retrieval pipeline with multiple test queries:

```bash
python retrieve.py
```

### 2. Test a Single Query

Test a specific query against the RAG system:

```bash
python retrieve.py --query "What are Vision-Language-Action (VLA) systems?"
```

### 3. Run with Verbose Logging

Enable detailed logging for debugging:

```bash
python retrieve.py --verbose
```

## 🌐 API Endpoints

The FastAPI backend exposes the following endpoints:

*   **`GET /`**: Root endpoint, returns a welcome message.
*   **`GET /health`**: Health check endpoint, returns status.
*   **`POST /chat`**: Main chat interface. Accepts a JSON body with `query` (string) and `max_chunks` (int). Returns the AI's response, status, and retrieved chunks.
*   **`GET /models`**: Returns information about the AI model being used.
*   **`POST /query_raw`**: Raw query endpoint, returns the full agent result without validation (for debugging).

## ⚠️ Troubleshooting

### Qdrant Connection Issues
*   Ensure Qdrant Docker container is running (`docker ps`).
*   Verify `QDRANT_URL` and `QDRANT_API_KEY` in your `.env` files.

### Cohere/Gemini API Errors
*   Check that `COHERE_API_KEY` and `GEMINI_API_KEY` are correctly set in your `.env` files.
*   Review API rate limits and your account quotas.

### Backend Not Starting
*   Ensure all Python dependencies are installed (`pip install -r requirements.txt`).
*   Check for syntax errors in Python files.
*   Verify `.env` file is correctly configured.

### Frontend Not Displaying Chatbot Correctly
*   Ensure the backend server is running and accessible at the `API_ENDPOINT` configured in `book_frontend/docusaurus.config.js`.
*   Check browser console for JavaScript errors.

## 📄 License

This project is licensed under the terms defined in the main project's licensing. (Please ensure a LICENSE file is present in the root of the repository if specific licensing is required.)
