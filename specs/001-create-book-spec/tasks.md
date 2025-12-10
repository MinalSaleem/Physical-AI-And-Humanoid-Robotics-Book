# Tasks: 001-create-book-spec

**Input**: Design documents from `/specs/001-create-book-spec/`
**Prerequisites**: plan.md, spec.md

## Docusaurus Setup Tasks

- [x] T001 Create `book/` directory in the root: `book/`
- [x] T002 Initialize Docusaurus within `book/` with TypeScript and classic preset: `cd book && npx create-docusaurus@latest . classic --typescript`
- [x] T003 Confirm initial Docusaurus files are generated in `book/` (e.g., `book/docusaurus.config.ts`, `book/src/pages/`)
- [x] T004 Ensure `docs`, `blog` (optional, remove if not needed), `pages` presets are configured in `book/docusaurus.config.ts`
- [x] T005 Verify MDX v3 support (default in Docusaurus 3.x) is present in `book/package.json`
- [x] T006 Utilize `@docusaurus/preset-classic` in `book/docusaurus.config.ts`
- [x] T007 Set `title` to "Physical AI & Humanoid Robotics" in `book/docusaurus.config.ts`
- [x] T008 Set `tagline` to "A Comprehensive Guide to Physical AI and Humanoid Robotics" in `book/docusaurus.config.ts`
- [x] T009 Set `url` to `https://[your-github-username].github.io` in `book/docusaurus.config.ts`
- [x] T010 Set `baseUrl` to `/Physical-AI-And-Humanoid-Robotics-Book/` in `book/docusaurus.config.ts`
- [x] T011 Set `organizationName` to `[your-github-username]` in `book/docusaurus.config.ts`
- [x] T012 Set `projectName` to `Physical-AI-And-Humanoid-Robotics-Book` in `book/docusaurus.config.ts`
- [x] T013 Set `deploymentBranch` to `gh-pages` in `book/docusaurus.config.ts`
- [x] T014 Set `onBrokenLinks` to `throw` in `book/docusaurus.config.ts`
- [x] T015 Set `onBrokenMarkdownLinks` to `warn` in `book/docusaurus.config.ts`
- [x] T016 Set `favicon` to `img/favicon.ico` in `book/docusaurus.config.ts`
- [x] T017 Configure `navbar` `title` and `logo` in `book/docusaurus.config.ts`
- [x] T018 Configure `navbar` `items` with links to docs, blog (if enabled), GitHub repo in `book/docusaurus.config.ts`
- [x] T019 Configure `footer` with standard copyright and links in `book/docusaurus.config.ts`
- [x] T020 Set `prism.theme` to `themes/github` in `book/docusaurus.config.ts`
- [x] T021 Set `prism.darkTheme` to `themes/dracula` in `book/docusaurus.config.ts`
- [x] T022 Ensure `darkMode` is enabled by default in `book/docusaurus.config.ts`
- [x] T023 Set `i18n` default to `en` in `book/docusaurus.config.ts`
- [x] T024 Enable docs versioning in `book/docusaurus.config.ts` (e.g., `docs: { lastVersion: 'current', versions: { current: { label: 'Next' } } }`)
- [x] T025 Set initial version `1.0.0` as default in Docusaurus versioning (if applicable based on config)
- [x] T026 Integrate Algolia DocSearch configuration in `book/docusaurus.config.ts`
- [x] T027 Configure `themeConfig.algolia` with `{ appId, apiKey, indexName, contextualSearch: true }` in `book/docusaurus.config.ts` (requires obtaining Algolia credentials)
- [x] T028 Integrate analytics (e.g., Google Analytics) via `themeConfig.googleAnalytics` or similar plugin in `book/docusaurus.config.ts`
- [x] T029 Configure auto-generated sidebar feature for `book/docs/` in `book/docusaurus.config.ts`
- [x] T030 Ensure `_category_.json` files are used within module and chapter directories to define labels and order in `book/docs/`

## Chapter Development Tasks (Repeatable Template)

The following tasks are template tasks to be repeated for each of the 4 modules and 4 chapters within each module (total 16 chapters).

### For each Module (e.g., Module 1):

- [ ] TXXX Create module directory: `book/docs/moduleN/` (e.g., `book/docs/module1/`)
- [ ] TXXX Create `_category_.json` for Module N: `book/docs/moduleN/_category_.json`
    ```json
    {
      "label": "Module N: [Module Title]",
      "position": N,
      "link": {
        "type": "generated-index",
        "title": "Module N Overview"
      }
    }
    ```

### For each Chapter within a Module (e.g., Chapter 1 in Module 1):

- [ ] TXXX Create chapter directory: `book/docs/moduleN/chapterM/` (e.g., `book/docs/module1/chapter1/`)
- [ ] TXXX Create `_category_.json` for Chapter M: `book/docs/moduleN/chapterM/_category_.json`
    ```json
    {
      "label": "Chapter M: [Chapter Title]",
      "position": M,
      "link": {
        "type": "generated-index",
        "title": "Chapter M Overview"
      }
    }
    ```
- [ ] TXXX Create `introduction.md` file for Chapter M: `book/docs/moduleN/chapterM/introduction.md`
- [ ] TXXX Create `key-concepts.md` file for Chapter M: `book/docs/moduleN/chapterM/key-concepts.md`
- [ ] TXXX Create `illustrative-examples.md` file for Chapter M: `book/docs/moduleN/chapterM/illustrative-examples.md`
- [ ] TXXX Create `exercises.md` file for Chapter M: `book/docs/moduleN/chapterM/exercises.md`
- [ ] TXXX Create `summary.md` file for Chapter M: `book/docs/moduleN/chapterM/summary.md`
