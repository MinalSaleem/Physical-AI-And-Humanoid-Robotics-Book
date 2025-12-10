# Implementation Plan: Book Specification for Physical AI and Humanoid Robotics

**Branch**: `001-create-book-spec` | **Date**: 2025-12-10 | **Spec**: [specs/001-create-book-spec/spec.md](specs/001-create-book-spec/spec.md)
**Input**: Feature specification from `/specs/001-create-book-spec/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan outlines the development of "Physical AI and Humanoid Robotics," a comprehensive and interactive open textbook built with Docusaurus. The book will feature a modular structure of 4 modules, each containing 4 chapters, interactive elements, and an integrated RAG chatbot powered by a backend service. The content size will be up to 100 pages or 50,000 words. Content entities will be identified by file path/URL slugs, and interactive elements will provide user-friendly error messages and guidance.

## Technical Context

**Language/Version**: TypeScript (`.tsx`, `.ts`) and Markdown (`.md`)  
**Primary Dependencies**: Docusaurus v3.x, Google Gemini (via Gemini API), Qdrant Cloud Free Tier, FastAPI (Python), Neon Serverless Postgres (free tier), OpenAI ChatKit or OpenAI Agents SDK  
**Storage**: Git repository for Docusaurus content and site, Qdrant Cloud for vector embeddings, Neon Serverless Postgres for FastAPI database/metadata.  
**Testing**: Component testing for critical UI elements.  
**Target Platform**: GitHub Pages  
**Project Type**: Web application (Docusaurus static site with integrated backend for RAG)  
**Performance Goals**: Lighthouse Performance: 90+, Accessibility: 95+  
**Constraints**: Total content up to 100 pages or 50,000 words.  
**Scale/Scope**: 4 modules, 4 chapters each, interactive elements, RAG chatbot.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

-   All tech stack choices align with constitution: **Pass**
-   Project goals align with constitution: **Pass**
-   Design system principles respected: **Pass**
-   Content structure aligns: **Pass**
-   Quality & Maintenance Laws considered: **Pass**
-   Success Metrics adhered to: **Pass**

## Project Structure

### Documentation (this feature)

```text
specs/001-create-book-spec/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
book/                   # Docusaurus project
├── docusaurus.config.js
├── src/
│   ├── css/
│   │   └── custom.css
│   ├── pages/
│   │   └── index.tsx
│   └── theme/
│       └── Root.tsx    # For RAG chatbot integration
├── docs/               # All book content
│   ├── module1/
│   │   ├── chapter1/
│   │   │   └── page1.md
│   │   └── ...
│   └── ...
├── static/             # Images, static assets
└── sidebar.js          # Or auto-generated
backend/                # FastAPI RAG service
├── main.py
├── requirements.txt
└── ...
```

**Structure Decision**: The project will adopt a split structure with a `book/` directory for the Docusaurus static site and a `backend/` directory for the FastAPI-based RAG chatbot service. This aligns with the "Option 2: Web application" pattern, separating frontend and backend concerns.

## Development Plan

### 1. Docusaurus Setup & Configuration

**Objective**: Initialize the Docusaurus project and configure its core settings, including required plugins, themes, and initial deployment readiness.

*   **1.1 Project Initialization**:
    *   Create a folder named `book` in the root directory.
    *   Initialize Docusaurus within the `book` folder:
        ```bash
        cd book
        npx create-docusaurus@latest . classic --typescript
        ```
    *   Confirm `docusaurus.config.ts`, `src/pages/`, `src/css/custom.css`, `static/`, `docs/`, `sidebars.js` (or similar) are generated.

*   **1.2 Required Plugins & Presets**:
    *   Ensure `docs`, `blog` (optional, can be removed if not needed), `pages` presets are configured.
    *   Verify MDX v3 support (default in Docusaurus 3.x).
    *   Utilize the `@docusaurus/preset-classic`.

*   **1.3 Core Configuration (`docusaurus.config.ts`)**:
    *   `title`: "Physical AI & Humanoid Robotics"
    *   `tagline`: "A Comprehensive Guide to Physical AI and Humanoid Robotics"
    *   `url`: `https://[your-github-username].github.io`
    *   `baseUrl`: `/Physical-AI-And-Humanoid-Robotics-Book/` (or repository name)
    *   `organizationName`: `[your-github-username]`
    *   `projectName`: `Physical-AI-And-Humanoid-Robotics-Book`
    *   `deploymentBranch`: `gh-pages`
    *   `onBrokenLinks`: `throw`
    *   `onBrokenMarkdownLinks`: `warn`
    *   `favicon`: `img/favicon.ico`
    *   `navbar`:
        *   `title`: "Physical AI & Humanoid Robotics"
        *   `logo`: `{alt: 'My Site Logo', src: 'img/logo.svg'}`
        *   `items`: Links to docs, blog (if enabled), GitHub repo.
    *   `footer`: Standard copyright, links.
    *   `prism.theme`: `themes/github`
    *   `prism.darkTheme`: `themes/dracula`
    *   `darkMode`: Enabled by default.
    *   `i18n`: Default to `en` (add configuration if multilingual support is planned later).

*   **1.4 Versioning Setup**:
    *   Enable docs versioning in `docusaurus.config.ts` (e.g., `docs: { lastVersion: 'current', versions: { current: { label: 'Next' } } }`)
    *   Initial version `1.0.0` will be the default.

*   **1.5 Search (Algolia DocSearch) & Analytics Setup**:
    *   Integrate Algolia DocSearch: Apply for a DocSearch account for credentials.
    *   Configure `themeConfig.algolia`: `{ appId, apiKey, indexName, contextualSearch: true }`.
    *   Integrate analytics (e.g., Google Analytics) via `themeConfig.googleAnalytics` or similar plugin.

### 2. Content Development Phases & Timeline

**Objective**: Structure the content creation process into manageable phases to ensure comprehensive coverage and quality.

| Phase | Description                                    | Duration | Dependencies |
|-------|------------------------------------------------|----------|--------------|
| 1     | **Skeleton + Sidebar**: Establish core book directory and navigation. | 1 week   | Docusaurus Setup |
| 2     | **Writing 16 Core Chapters**: Draft all module and chapter content.   | 8 weeks  | Phase 1        |
| 3     | **Assets, Diagrams, Review, Exercises**: Create visual aids, conduct peer reviews, finalize exercises. | 4 weeks  | Phase 2        |
| 4     | **Polish, Accessibility, SEO, Mobile Testing**: Refine content, ensure compliance, optimize for web. | 2 weeks  | Phase 3        |
| 5     | **Launch and Post-Launch Updates**: Publish the book, monitor, and plan for future editions. | Ongoing  | Phase 4        |

### 3. Exact File & Folder Structure

**Objective**: Define a clear and consistent file and folder structure for the book's content and assets.

*   **3.1 Proposed Folder Layout under `/docs`**:
    ```text
    book/docs/
    ├── module1/
    │   ├── chapter1/
    │   │   ├── _category_.json  # Optional: For custom sidebar labels/positions
    │   │   ├── introduction.md
    │   │   ├── key-concepts.md
    │   │   ├── illustrative-examples.md
    │   │   ├── exercises.md
    │   │   └── summary.md
    │   ├── chapter2/
    │   └── ... (up to chapter4)
    ├── module2/
    │   └── ...
    ├── module3/
    │   └── ...
    └── module4/
        └── ...
    ```

*   **3.2 Precise Sidebar Configuration (`sidebar.js` or Auto-generated)**:
    *   Utilize Docusaurus's auto-generated sidebar feature for `docs` folder.
    *   Ensure `_category_.json` files are used within module and chapter directories to define labels and order.
    *   Example `_category_.json` for `book/docs/module1/chapter1/`:
        ```json
        {
          "label": "Chapter 1: [Chapter Title]",
          "position": 1,
          "link": {
            "type": "generated-index",
            "title": "Chapter 1 Overview"
          }
        }
        ```

*   **3.3 Naming Conventions for Modules, Chapters, and Assets**:
    *   **Modules**: `module[number]/` (e.g., `module1/`)
    *   **Chapters**: `chapter[number]/` (e.g., `chapter1/`)
    *   **Content Files**: `kebab-case.md` (e.g., `introduction.md`, `key-concepts.md`).
    *   **Assets**: `kebab-case.png`, `kebab-case.mp4` for images/videos.

*   **3.4 How to Handle Images, Videos, Interactive Demos, and Bibliography**:
    *   **Images**: Stored in `book/static/img/` or within chapter-specific `img/` subdirectories for better organization. Referenced via relative paths (`![Alt Text](../../static/img/image.png)` or `![Alt Text](./img/image.png)`).
    *   **Videos**: Store video files in `book/static/videos/` and embed using Markdown or custom React components for advanced players (e.g., `import VideoPlayer from '@site/src/components/VideoPlayer'; <VideoPlayer src="/videos/my-video.mp4" />`).
    *   **Interactive Demos**: Implement as React components within `book/src/components/` and embed directly into MDX files.
    *   **Bibliography**: Use Docusaurus plugin for citations (e.g., `docusaurus-plugin-content-docs` with custom fields) or a dedicated Markdown file under `book/docs/references.md`.

## Research

### Docusaurus Documentation and Best Practices

To ensure the plan adheres to the latest Docusaurus best practices and recommended configurations, a thorough review of the official Docusaurus documentation (v3.x) is essential. Key areas of focus include:

*   **Initialization and Project Structure**: Understanding the `npx create-docusaurus` command options and the recommended file layout.
*   **Configuration (`docusaurus.config.ts`)**: Deep dive into `themeConfig`, `presets`, `plugins`, and `navbar`/`footer` customization.
*   **Content Management**: Best practices for Markdown/MDX, handling images, and linking.
*   **Version Management**: Detailed setup for documentation versioning.
*   **Search Integration**: Latest recommendations for Algolia DocSearch.
*   **Deployment**: Specifics for GitHub Pages deployment.
*   **Accessibility and SEO**: Docusaurus's built-in features and best practices for optimization.
*   **Interactive Elements**: Guidance on integrating custom React components within MDX.

This research will be continuous throughout Phase 0 and Phase 1 to ensure decisions are informed by the most up-to-date and idiomatic Docusaurus approaches.