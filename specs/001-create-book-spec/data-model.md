# Data Model: Book Specification for Physical AI and Humanoid Robotics

**Branch**: `001-create-book-spec` | **Date**: 2025-12-10 | **Spec**: [specs/001-create-book-spec/spec.md](specs/001-create-book-spec/spec.md)

## Entities

### Module

*   **Description**: A top-level organizational unit of the book, encapsulating a major theme or area of physical AI and humanoid robotics.
*   **Attributes**:
    *   `id`: `string` (Unique identifier, derived from file path/URL slug, e.g., `module1`)
    *   `title`: `string` (Descriptive title of the module)
    *   `description`: `string` (2-4 paragraphs summarizing key topics, objectives, and learning outcomes for the module)
    *   `chapters`: `list<Chapter>` (Collection of Chapter entities belonging to this module)
*   **Relationships**: Contains multiple `Chapter` entities.
*   **Validation Rules**:
    *   MUST contain exactly 4 `Chapter` entities.
    *   `id` MUST be unique across all modules.

### Chapter

*   **Description**: A secondary organizational unit within a module, focusing on a specific topic or concept.
*   **Attributes**:
    *   `id`: `string` (Unique identifier, derived from file path/URL slug, e.g., `chapter1`)
    *   `title`: `string` (Descriptive title of the chapter)
    *   `description`: `string` (2-4 paragraphs summarizing key topics, objectives, and learning outcomes for the chapter)
    *   `content_pages`: `list<Content Page>` (Collection of Content Page entities forming the chapter's content)
*   **Relationships**: Belongs to a `Module`, contains multiple `Content Page` entities.
*   **Validation Rules**:
    *   MUST contain an "Introduction", "Key Concepts", "Illustrative Examples" (pseudocode only), "Exercises", and a "Summary" as core content.
    *   `id` MUST be unique within its parent module.

### Content Page

*   **Description**: An individual Markdown file that forms a section or lesson within a chapter, designed to be self-contained but interconnected with other topics.
*   **Attributes**:
    *   `id`: `string` (Unique identifier, derived from file path/URL slug, e.g., `introduction.md`)
    *   `title`: `string` (Page title, typically from Markdown frontmatter)
    *   `body`: `string` (Markdown content of the page)
    *   `frontmatter`: `object` (Consistent metadata including `id`, `title`, `description`, etc.)
*   **Relationships**: Belongs to a `Chapter`.
*   **Validation Rules**:
    *   MUST adhere to clear, modular Markdown formatting.
    *   All images MUST include `alt` text.
    *   MUST use proper heading hierarchy (H1 → H6).
    *   `id` MUST be unique within its parent chapter.