#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

python3 tools/validate_documentation_content.py

if ! command -v doxygen >/dev/null 2>&1; then
    echo "doxygen is required to build the dtDynamics manual." >&2
    exit 2
fi

if ! command -v dot >/dev/null 2>&1; then
    echo "Graphviz dot is required for manual diagrams." >&2
    exit 2
fi

output_dir="${DTDYNAMICS_DOCS_OUTPUT_DIR:-${repo_root}/build/docs/doxygen}"
output_dir="$(python3 - "${output_dir}" <<'PYPATH'
from pathlib import Path
import sys
print(Path(sys.argv[1]).expanduser().resolve(strict=False))
PYPATH
)"
if [[ -z "${output_dir}" || "${output_dir}" == "/" ]]; then
    echo "Unsafe documentation output directory after canonicalization: ${output_dir}" >&2
    exit 2
fi
case "${output_dir}" in
    */docs/doxygen) ;;
    *)
        echo "Documentation output must end in /docs/doxygen: ${output_dir}" >&2
        exit 2
        ;;
esac
mkdir -p "${output_dir}"

work_dir="$(mktemp -d "${TMPDIR:-/tmp}/dtdynamics-docs-validate.XXXXXX")"
cleanup()
{
    rm -rf "${work_dir}"
}
trap cleanup EXIT

for dot_file in docs/manual/diagrams/*.dot; do
    dot -Tsvg "${dot_file}" -o "${work_dir}/$(basename "${dot_file}" .dot).svg"
done

python3 - "${repo_root}/Doxyfile" "${work_dir}/Doxyfile" "${output_dir}" <<'PYCONFIG'
from pathlib import Path
import sys
source, destination, output_dir = map(Path, sys.argv[1:])
lines = source.read_text().splitlines()
for index, line in enumerate(lines):
    if line.lstrip().startswith("OUTPUT_DIRECTORY"):
        lines[index] = f"OUTPUT_DIRECTORY       = {output_dir}"
        break
else:
    raise SystemExit("Doxyfile is missing OUTPUT_DIRECTORY")
destination.write_text("\n".join(lines) + "\n")
PYCONFIG

# Do not recursively delete caller-selected paths. Doxygen overwrites its owned
# files, and pipefail makes a failed generation fail before the index assertion.
log_path="${output_dir}/doxygen.log"
doxygen "${work_dir}/Doxyfile" 2>&1 | tee "${log_path}"
if grep -Eiq '(^|[[:space:]])(warning|error):' "${log_path}"; then
    echo "FAIL: Doxygen emitted warnings or errors. See ${log_path}" >&2
    exit 1
fi
test -s "${output_dir}/html/index.html"
printf 'PASS: Doxygen manual generated at %s\n' "${output_dir}/html/index.html"
