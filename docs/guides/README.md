# Presentation guides

This directory is the canonical editable source for the dtDynamics presentation guides that are distributed as PDFs to downstream controller repositories.

## Contents

- `model_guide/mjcf-xml-dtdynspec-dtdynmodel-guide.md`: MJCF-like XML, `dtDynSpec`, `dtDynModel`, `dtDynData`, and structural update guide.
- `usage_guide/dtdynamics-usage-migration-guide.md`: package usage, algorithms, LeoQuad migration, validation, and benchmark guide.
- `themes/hmg.css`: shared Marp theme.

Referenced SVG files stay beside each guide under its `figures/` directory. Generated HTML, PPTX, and PDF files are not committed here. The current package manual is under `docs/manual/`; benchmark numbers and scope definitions are authoritative in `docs/benchmark-results.md`.

## Build

From the repository root:

```bash
marp --allow-local-files \
  --theme docs/guides/themes/hmg.css \
  --pdf \
  docs/guides/model_guide/mjcf-xml-dtdynspec-dtdynmodel-guide.md \
  -o /tmp/mjcf-xml-dtdynspec-dtdynmodel-guide.pdf

marp --allow-local-files \
  --theme docs/guides/themes/hmg.css \
  --pdf \
  docs/guides/usage_guide/dtdynamics-usage-migration-guide.md \
  -o /tmp/dtdynamics-usage-migration-guide.pdf
```

Run Marp from the repository root so relative image references resolve through each Markdown file's directory. Before distributing rebuilt PDFs, verify page count, embedded text, local images, and visual overflow.

## Downstream distribution

`leoquad-control/docs/dtdynamics/` is a PDF-only distribution location. Copy only these rebuilt outputs there:

- `mjcf-xml-dtdynspec-dtdynmodel-guide.pdf`
- `dtdynamics-usage-migration-guide.pdf`

Do not edit downstream copies or restore stale package manuals, benchmark reports, paper drafts, source decks, backups, HTML, or PPTX files there.
