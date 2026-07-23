# Presentation guides

This directory publishes the dtDynamics presentation guides as PDFs for direct viewing in GitLab. The editable Marp slide Markdown files are maintained locally and intentionally ignored so they are not displayed in the repository.

## Published guides

- [`model_guide/mjcf-xml-dtdynspec-dtdynmodel-guide.pdf`](model_guide/mjcf-xml-dtdynspec-dtdynmodel-guide.pdf): MJCF-like XML, `dtDynSpec`, `dtDynModel`, `dtDynData`, and structural update guide.
- [`usage_guide/dtdynamics-usage-migration-guide.pdf`](usage_guide/dtdynamics-usage-migration-guide.pdf): package usage, algorithms, LeoQuad migration, validation, and benchmark guide.

The shared theme and referenced SVG assets remain tracked under `themes/` and each guide's `figures/` directory. The current package manual is under `docs/manual/`; benchmark numbers and scope definitions are authoritative in `docs/benchmark-results.md`.

## Local rebuild

The following ignored source files must exist in the maintainer worktree:

- `docs/guides/model_guide/mjcf-xml-dtdynspec-dtdynmodel-guide.md`
- `docs/guides/usage_guide/dtdynamics-usage-migration-guide.md`

From the repository root, rebuild the tracked PDFs with:

```bash
npx --yes @marp-team/marp-cli \
  docs/guides/model_guide/mjcf-xml-dtdynspec-dtdynmodel-guide.md \
  --pdf --allow-local-files --theme=docs/guides/themes/hmg.css \
  -o /tmp/dtdyn-model-guide.pdf

npx --yes @marp-team/marp-cli \
  docs/guides/usage_guide/dtdynamics-usage-migration-guide.md \
  --pdf --allow-local-files --theme=docs/guides/themes/hmg.css \
  -o /tmp/dtdyn-usage-guide.pdf

install -m 0644 /tmp/dtdyn-model-guide.pdf \
  docs/guides/model_guide/mjcf-xml-dtdynspec-dtdynmodel-guide.pdf
install -m 0644 /tmp/dtdyn-usage-guide.pdf \
  docs/guides/usage_guide/dtdynamics-usage-migration-guide.pdf
```

Before committing rebuilt PDFs, verify page counts, embedded text, local images, and visual overflow.

## Downstream distribution

`leoquad-control/docs/dtdynamics/` remains a PDF-only distribution location. Copy only the two published PDFs there; do not restore source decks, package manuals, benchmark reports, paper drafts, backups, HTML, or PPTX files.
