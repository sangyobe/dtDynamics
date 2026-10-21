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

The release package is a source archive generated only from the exact vVERSION
tag that resolves to the current clean git HEAD. The requested archive/package
version must match the project version in CMakeLists.txt. By default, the full
release-readiness audit runs before the archive is created.
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

cmake_version="$(sed -n -E 's/.*[[:space:]]VERSION[[:space:]]+([0-9]+\.[0-9]+\.[0-9]+).*/\1/p' CMakeLists.txt | head -n 1)"
if [[ -z "${cmake_version}" ]]; then
    echo "Could not determine the project version from CMakeLists.txt." >&2
    exit 1
fi

if [[ -z "${VERSION}" ]]; then
    VERSION="${cmake_version}"
fi
if [[ "${VERSION}" != "${cmake_version}" ]]; then
    echo "Release version ${VERSION} does not match project version ${cmake_version}." >&2
    exit 1
fi

safe_version="$(printf '%s' "${VERSION}" | sed 's/[^A-Za-z0-9._+-]/-/g')"
if [[ "${safe_version}" != "${VERSION}" ]]; then
    echo "Release version contains unsupported archive characters: ${VERSION}" >&2
    exit 1
fi
release_tag="v${VERSION}"

head_commit="$(git rev-parse HEAD)"
if ! tag_commit="$(git rev-parse --verify -q "refs/tags/${release_tag}^{commit}")"; then
    echo "Missing exact release tag ${release_tag}; create and verify it before packaging." >&2
    exit 1
fi
if [[ "${tag_commit}" != "${head_commit}" ]]; then
    echo "Exact release tag ${release_tag} does not resolve to HEAD." >&2
    echo "tag=${tag_commit}" >&2
    echo "HEAD=${head_commit}" >&2
    exit 1
fi

if [[ "${RUN_AUDIT}" == "ON" ]]; then
    "${SCRIPT_DIR}/audit_release_readiness.sh"
fi

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

archive_tmp="${archive_path}.tmp.$$"
archive_tar_tmp="${archive_path}.tar.tmp.$$"
trap 'rm -f "${archive_tmp}" "${archive_tar_tmp}"' EXIT

git archive \
    --format=tar.gz \
    --prefix="dtDynamics-${safe_version}/" \
    -o "${archive_tmp}" \
    "${tag_commit}"

# Decompress to a file instead of piping into `git get-tar-commit-id`: that command reads only
# the tar header and exits, so on an archive larger than a pipe buffer `gzip` is killed by
# SIGPIPE and `set -o pipefail` aborts the release with status 141.
gzip -dc -- "${archive_tmp}" > "${archive_tar_tmp}"
archive_commit="$(git get-tar-commit-id < "${archive_tar_tmp}")"
rm -f "${archive_tar_tmp}"
if [[ "${archive_commit}" != "${tag_commit}" ]]; then
    echo "Generated source archive does not identify the exact release tag commit." >&2
    echo "archive=${archive_commit:-missing}" >&2
    echo "tag=${tag_commit}" >&2
    exit 1
fi
mv "${archive_tmp}" "${archive_path}"
trap - EXIT

(
    cd "${OUTPUT_DIR}"
    sha256sum "${archive_name}" > "${archive_name}.sha256"
)

echo "PASS: dtDynamics release archive created."
echo "Archive: ${archive_path}"
echo "Checksum: ${checksum_path}"
