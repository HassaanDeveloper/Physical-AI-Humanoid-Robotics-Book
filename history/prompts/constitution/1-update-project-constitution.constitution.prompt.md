---
id: 1
title: "update-project-constitution"
stage: constitution
date: "2026-01-08"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "none"
branch: "master"
user: "user"
command: "/sp.constitution"
labels: ["constitution", "spec-driven-development", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - ".specify/memory/constitution.md"
tests: []
---

# Constitution Update PHR

## Prompt Text

```
/sp.constitution

Project: Spec-Driven AI Book with Embedded RAG Chatbot

Objective:
Create a technical book using Claude Code and Spec-Kit Plus, publish it with Docusaurus on GitHub Pages, and embed a RAG chatbot that answers questions strictly from the book content, including user-selected text only.

Core Principles:
- Spec-first development (no implementation without specification)
- Grounded responses (no hallucinations)
- Traceability between specs, content, and code
- Modular and reproducible architecture
- Production-grade quality

Book Standards:
- Tools: Claude Code + Spec-Kit Plus
- Framework: Docusaurus
- Audience: Software engineers / CS students
- Style: Clear, technical, example-driven
- Rules:
  - No uncited factual claims
  - Code must be runnable or marked pseudocode
  - Consistent structure and metadata

RAG Chatbot Standards:
- Stack:
  - OpenAI Agents / ChatKit SDKs
  - FastAPI
  - Neon Serverless Postgres
  - Qdrant Cloud (Free Tier)
- Modes:
  1. Full-book retrieval
  2. User-selected t
```

## Response Text

Updated the project constitution with the specified principles for the Spec-Driven AI Book with Embedded RAG Chatbot project. The new constitution includes:

- Core Principles: Spec-first development, Grounded responses, Traceability, Modular architecture, Production-grade quality, and Book content standards
- Technology Stack Requirements with the specified tools and frameworks
- Development Workflow aligned with Spec-Driven Development practices
- Governance section with compliance requirements

## Outcome

The constitution has been successfully created at `.specify/memory/constitution.md` with version 1.0.0. The file contains all the principles and requirements specified in the user input, structured according to the constitution template.

## Evaluation

The constitution update was successful, with all placeholder tokens replaced with concrete values based on the user's requirements. The document is ready for use as the governing document for the project.