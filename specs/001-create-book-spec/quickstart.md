# Quickstart Guide: Physical AI and Humanoid Robotics Book

**Branch**: `001-create-book-spec` | **Date**: 2025-12-10 | **Spec**: [specs/001-create-book-spec/spec.md](specs/001-create-book-spec/spec.md)

This guide provides instructions to quickly set up and run the "Physical AI and Humanoid Robotics" Docusaurus book and its associated RAG (Retrieval-Augmented Generation) chatbot backend.

## Prerequisites

Before you begin, ensure you have the following installed on your system:

*   **Node.js** (LTS version, e.g., v18.x or v20.x)
*   **npm** (usually comes with Node.js) or **Yarn** (recommended by Docusaurus)
*   **Python 3.9+**
*   **pip** (Python package installer)
*   **Git**

## 1. Clone the Repository

First, clone the project repository from GitHub:

```bash
git clone https://github.com/[your-github-username]/Physical-AI-And-Humanoid-Robotics-Book.git
cd Physical-AI-And-Humanoid-Robotics-Book
```

## 2. Docusaurus Book Setup

Navigate into the `book` directory and install its dependencies:

```bash
cd book
npm install # or yarn install
```

To start the Docusaurus development server:

```bash
npm run start # or yarn start
```

This will open the book in your browser at `http://localhost:3000`. Any changes you make to the Markdown content or Docusaurus configuration will automatically reload the page.

## 3. RAG Chatbot Backend Setup

Open a **new terminal window** and navigate to the project's root directory. Then, enter the `backend` directory:

```bash
cd backend
```

Create a Python virtual environment and activate it:

```bash
python -m venv venv
# On Windows
.\venv\Scripts\activate
# On macOS/Linux
source venv/bin/activate
```

Install the required Python packages:

```bash
pip install -r requirements.txt
```

Create a `.env` file in the `backend` directory based on a `.env.example` (you may need to create this example file first with placeholder values for your Gemini API Key, Qdrant details, and Neon Postgres connection string).

```dotenv
# .env file example for backend
GEMINI_API_KEY="YOUR_GOOGLE_GEMINI_API_KEY"
QDRANT_URL="YOUR_QDRANT_CLOUD_URL"
QDRANT_API_KEY="YOUR_QDRANT_API_KEY"
DATABASE_URL="YOUR_NEON_POSTGRES_CONNECTION_STRING"
```

Start the FastAPI application:

```bash
uvicorn main:app --reload
```

This will start the backend server, typically at `http://localhost:8000`. You can access the FastAPI interactive API documentation (Swagger UI) at `http://localhost:8000/docs`.

## 4. Interacting with the Chatbot

Once both the Docusaurus book and the FastAPI backend are running, the RAG chatbot widget within the book should become active. You can open the book in your browser (`http://localhost:3000`) and interact with the chatbot as described in the book's interface.

*(Note: Initial setup of the RAG system, including content ingestion, chunking, embedding, and indexing into Qdrant, will be handled separately and is not part of this quickstart guide for running the local development environment. Ensure your Qdrant instance is populated with book embeddings for the chatbot to function effectively.)*
