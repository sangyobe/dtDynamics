#!/usr/bin/env bash
# Side-effect-free clang-format gate for the dtDynamics production source tree.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
PACKAGE_CONFIG_TEST="${REPO_ROOT}/tools/test_dtdynamics_config_include_order.py"

mkdir -p reports

SEARCH_DIRS=()
for dir in include src; do
    if [[ -d "${dir}" ]]; then
        SEARCH_DIRS+=("${dir}")
    fi
done

if [[ ${#SEARCH_DIRS[@]} -eq 0 ]]; then
    echo "ERROR: clang-format scope directories include/ and src/ are missing"
    exit 1
fi

if ! command -v clang-format >/dev/null 2>&1; then
    echo "ERROR: clang-format is not available in PATH"
    exit 1
fi

FILE_LIST="$(mktemp "${TMPDIR:-/tmp}/dtdynamics-clang-format.XXXXXX")" || {
    echo "ERROR: cannot create clang-format source list"
    exit 1
}
trap 'rm -f "${FILE_LIST}"' EXIT

if ! find "${SEARCH_DIRS[@]}" \
    -type f \
    \( -name "*.cpp" -o -name "*.h" \) \
    -print0 | sort -z > "${FILE_LIST}"
then
    echo "ERROR: clang-format source discovery failed"
    exit 1
fi
mapfile -d '' FILES < "${FILE_LIST}"

if [[ ${#FILES[@]} -eq 0 ]]; then
    echo "ERROR: no C/C++ files found in clang-format scope"
    exit 1
fi

echo "=== dtDynamics clang-format check ==="
echo "Target roots: ${SEARCH_DIRS[*]}"
printf 'Files: %s\n' "${#FILES[@]}"

set +e
clang-format --dry-run --Werror "${FILES[@]}" 2>&1 | tee reports/clang-format.txt
PIPE_STATUS=("${PIPESTATUS[@]}")
set -e

FORMAT_EXIT=${PIPE_STATUS[0]:-1}
TEE_EXIT=${PIPE_STATUS[1]:-1}
if [[ ${FORMAT_EXIT} -eq 0 && ${TEE_EXIT} -eq 0 ]]; then
    echo "clang-format check passed"
    echo "=== dtDynamics package-config include precedence ==="
    python3 "${PACKAGE_CONFIG_TEST}"
    exit 0
fi

if [[ ${TEE_EXIT} -ne 0 ]]; then
    echo "ERROR: could not write reports/clang-format.txt"
fi
if [[ ${FORMAT_EXIT} -ne 0 ]]; then
    echo "clang-format violations found. Run: bash ci/clang-format.sh --fix"
    exit "${FORMAT_EXIT}"
fi
exit "${TEE_EXIT}"
