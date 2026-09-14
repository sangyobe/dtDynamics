#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
command -v dot >/dev/null 2>&1 || { echo "dot (Graphviz) is required" >&2; exit 2; }
for source in "${root}"/docs/manual/diagrams/*.dot; do
  dot -Tsvg "${source}" -o "${source%.dot}.svg"
done
printf 'PASS: Markdown diagram SVGs regenerated\n'
