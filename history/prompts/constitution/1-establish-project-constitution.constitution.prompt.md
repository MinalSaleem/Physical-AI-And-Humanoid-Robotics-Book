---
id: 1
title: Establish Project Constitution
stage: constitution
date: 2025-12-10
surface: agent
model: gemini-pro
feature: none
branch: main
user: User
command: /sp.constitution
labels: []
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
tests:
- none
---

## Prompt

/sp.constitution # Establish Project Constitution

Create a Constitution `constitution.md` document for a book about "Physical AI & Humanoid Robotics" with these requirements

## 1. Project Definition:
   * **Title**: [Your Book Title - "Physical AI & Humanoid Robotics"]
   * **Goal**: Create a comprehensive, interactive book using Docusaurus that includes an embedded RAG chatbot for enhanced learning and content discovery.
   * **Audience**: Srudents from intermediate level to advanced
   * **Delivery**: Published as a static site on GitHub Pages with full search and AI assistance capabilities.

## 2. Tech Stack Laws:
   * **Framework**: Docusaurus (Latest stable version - v3.x)
   * **Language**: TypeScript (`.tsx`, `.ts`) and Markdown (`.md`) for content
   * **Styling**: 
     - CSS Modules 
     - Global theme variables in `src/css/custom.css`
   * **RAG Chatbot Stack**:
        **AI Model**: Google Gemini (via Gemini API)
        **Vector Store**: Qdrant Cloud Free Tier
        **Backend**: FastAPI (Python)
        **Database**: Neon Serverless Postgres (free tier)
        **Agents Framework**: OpenAI ChatKit or OpenAI Agents SDK
   * **Build & Deployment**:
     - GitHub Actions for CI/CD
     - GitHub Pages for hosting
     - Spec-Kit Plus for documentation standards

## 3. Design System:
   * **Visual Identity**:
     - Professional, clean, and modern aesthetic
   * **Color Palette**:
     - Primary: [Define primary color - e.g., `#334155` (Slate)]
     - Secondary: [Define secondary color - e.g., `#64748b` (Cool Gray)]
     - Background: [Light/Dark theme colors]
     - Text: High contrast ratios for accessibility
   * **UI Components**:
     - Consistent navigation (sidebar, navbar, footer)
     - Interactive elements (tabs, code blocks with copy buttons)
     - Responsive design
     - Chat widget: Fixed position (bottom-right), collapsible, accessible

## 4. Content Structure:
   * **Organization**:
     - Logical chapter/section hierarchy
     - Progressive difficulty curve
     - Modular content (each page self-contained but interconnected)
   * **Content Requirements**:
     - Book contains 4 modules
     - Each modules includes:
       - 4 chapters in each module
       - Clear learning objectives
       - Practical examples with code snippets
       - Further reading/resources
   * **Documentation Standards** (Spec-Kit Plus):
     - Consistent frontmatter metadata
     - Proper heading hierarchy (H1 → H6)
     - Code block language tags
     - Alt text for all images
     - Internal linking between related topics

## 5. RAG Chatbot Architecture:
   * **Core Functionality**:
     - Context-aware responses based on book content
     - Source citation (links to relevant pages)
     - Conversation memory within session
     - Fallback to general Gemini knowledge when needed
   * **Integration Pattern**:
     - Embedded via `src/theme/Root.tsx` (Docusaurus wrapper pattern)
     - React component with state management
     - API calls to Gemini with retrieval context
   * **Data Pipeline**:
     - **Ingestion**: Automated extraction of all `.md`/`.mdx` content
     - **Chunking**: Break content into semantic chunks (overlap strategy)
     - **Embedding**: Generate vector embeddings for each chunk
     - **Indexing**: Store in vector database with metadata (page title, URL, section)
     - **Retrieval**: Semantic search on user queries
     - **Generation**: Gemini generates answers using retrieved context
   * **User Experience**:
     - Chat widget toggleable via button
     - Loading states and error handling
     - Rate limiting and usage tracking

## 6. Deployment & CI/CD:
   * **Version Control**:
     - Git branching strategy (main, develop, feature branches)
     - Conventional commits for changelog generation
   * **Automated Workflows** (GitHub Actions):
     - Build validation on pull requests
     - Automated deployment to GitHub Pages on merge to main
     - RAG index regeneration on content updates
   * **Performance**:
     - Static site optimization (image compression, lazy loading)
     - CDN caching strategies
     - Lighthouse score targets (Performance: 90+, Accessibility: 95+)

## 7. Quality & Maintenance Laws:
   * **Code Quality**:
     - TypeScript strict mode enabled
     - ESLint and Prettier for code consistency
     - Component testing for critical UI elements
   * **Content Quality**:
     - Peer review process for new chapters
     - Regular content audits for accuracy
     - Dead link checking (automated)
   * **Security**:
     - API keys stored in environment variables (never committed)
     - Rate limiting on chatbot API calls
     - Content Security Policy (CSP) headers
   * **Accessibility**:
     - Keyboard navigation support
     - Color contrast compliance

## 8. Success Metrics:
   * **Content Coverage**:
     - RAG retrieval accuracy (90% of relevant answers)
     * Chapter completion status

## Response snapshot

I have created the `constitution.md` file in `.specify/memory/constitution.md` and the corresponding PHR in `history/prompts/constitution/1-establish-project-constitution.constitution.prompt.md`.

## Outcome

- ✅ Impact: The project constitution has been established, providing a guiding framework for the project.
- 🧪 Tests: No tests were run as this was a documentation task.
- 📁 Files: 
  - `.specify/memory/constitution.md` (created)
  - `history/prompts/constitution/1-establish-project-constitution.constitution.prompt.md` (created)
- 🔁 Next prompts: The user can now proceed with other commands, such as `/sp.plan` or `/sp.specify`.
- 🧠 Reflection: The process was smooth. Creating the directories first and then the file is a reliable workflow.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): None
