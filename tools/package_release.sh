#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DTDYNAMICS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

OUTPUT_DIR="${DTDYNAMICS_RELEASE_DIR:-${DTDYNAMICS_ROOT}/dist}"
VERSION="${DTDYNAMICS_RELEASE_VERSION:-}"
RUN_AUDIT=ON
FORCE=OFF

usage()
{
    cat <<USAGE
Usage: tools/package_release.sh [options]

Options:
  --version VERSION      Release version string for the archive name.
  --output-dir DIR       Directory for the generated archive and checksum.
  --skip-audit           Do not run tools/audit_release_readiness.sh first.
  --force                Replace an existing archive with the same name.
  -h, --help             Show this help text.

Environment:
  DTDYNAMICS_RELEASE_DIR      Default output directory.
  DTDYNAMICS_RELEASE_VERSION  Default version string.

The release package is a source archive generated from git HEAD. The working
tree must be clean so the archive is exactly reproducible from the committed
source state. By default, the full release-readiness audit runs before the
archive is created.
USAGE
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --version)
            if [[ $# -lt 2 ]]; then
                echo "--version requires an argument" >&2
                exit 1
            fi
            VERSION="$2"
            shift
            ;;
        --output-dir)
            if [[ $# -lt 2 ]]; then
                echo "--output-dir requires an argument" >&2
                exit 1
            fi
            OUTPUT_DIR="$2"
            shift
            ;;
        --skip-audit)
            RUN_AUDIT=OFF
            ;;
        --force)
            FORCE=ON
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
    shift
done

cd "${DTDYNAMICS_ROOT}"

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "dtDynamics release packaging requires a git work tree." >&2
    exit 1
fi

if [[ -n "$(git status --porcelain)" ]]; then
    echo "Working tree is dirty. Commit or stash changes before packaging." >&2
    git status --short >&2
    exit 1
fi

if [[ "${RUN_AUDIT}" == "ON" ]]; then
    "${SCRIPT_DIR}/audit_release_readiness.sh"
fi

if [[ -z "${VERSION}" ]]; then
    cmake_version="$(sed -n 's/^[[:space:]]*VERSION[[:space:]]\+\([^[:space:]]\+\).*/\1/p' CMakeLists.txt | head -n 1)"
    git_sha="$(git rev-parse --short=12 HEAD)"
    VERSION="${cmake_version:-0.0.0}+g${git_sha}"
fi

safe_version="$(printf '%s' "${VERSION}" | sed 's/[^A-Za-z0-9._+-]/-/g')"
archive_name="dtDynamics-${safe_version}.tar.gz"
archive_path="${OUTPUT_DIR}/${archive_name}"
checksum_path="${archive_path}.sha256"

mkdir -p "${OUTPUT_DIR}"

if [[ "${FORCE}" == "OFF" && -e "${archive_path}" ]]; then
    echo "Archive already exists: ${archive_path}" >&2
    echo "Use --force to replace it." >&2
    exit 1
fi

if [[ "${FORCE}" == "ON" ]]; then
    rm -f "${archive_path}" "${checksum_path}"
fi

git archive \
    --format=tar.gz \
    --prefix="dtDynamics-${safe_version}/" \
    -o "${archive_path}" \
    HEAD

(
    cd "${OUTPUT_DIR}"
    sha256sum "${archive_name}" > "${archive_name}.sha256"
)

echo "PASS: dtDynamics release archive created."
echo "Archive: ${archive_path}"
echo "Checksum: ${checksum_path}"
