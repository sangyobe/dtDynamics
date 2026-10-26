#!/usr/bin/env bash
set -euo pipefail

: "${CI_PROJECT_DIR:?CI_PROJECT_DIR is required}"
: "${CI_SERVER_URL:?CI_SERVER_URL is required}"
: "${CI_JOB_TOKEN:?CI_JOB_TOKEN is required for the immutable dtMath checkout}"
: "${DTDYNAMICS_DTMATH_COMMIT:?DTDYNAMICS_DTMATH_COMMIT is required}"
: "${DTDYNAMICS_DTMATH_INCLUDE_DIR:?DTDYNAMICS_DTMATH_INCLUDE_DIR is required}"

ci_root="${CI_PROJECT_DIR}/.ci"
checkout="${ci_root}/dtmath"
expected_include="${checkout}/include"
repository="${CI_SERVER_URL%/}/rlab/ardg/arct/project/arch/03-art-framework/dtmath/dtmath.git"

if [[ "${DTDYNAMICS_DTMATH_INCLUDE_DIR}" != "${expected_include}" ]]; then
    echo "ERROR: DTDYNAMICS_DTMATH_INCLUDE_DIR must select the job-local immutable checkout" >&2
    exit 2
fi

askpass="$(mktemp)"
cleanup() {
    rm -f "${askpass}"
}
trap cleanup EXIT

cat >"${askpass}" <<'EOF'
#!/usr/bin/env bash
set -eu
case "${1:-}" in
    *Username*) printf '%s\n' 'gitlab-ci-token' ;;
    *Password*) printf '%s\n' "${CI_JOB_TOKEN:?}" ;;
    *) exit 1 ;;
esac
EOF
chmod 700 "${askpass}"

rm -rf "${checkout}"
mkdir -p "${ci_root}"
GIT_ASKPASS="${askpass}" GIT_TERMINAL_PROMPT=0 \
    git clone --no-checkout "${repository}" "${checkout}"
GIT_ASKPASS="${askpass}" GIT_TERMINAL_PROMPT=0 \
    git -C "${checkout}" fetch --depth=1 origin "${DTDYNAMICS_DTMATH_COMMIT}"
git -C "${checkout}" checkout --detach "${DTDYNAMICS_DTMATH_COMMIT}"

actual_commit="$(git -C "${checkout}" rev-parse HEAD)"
if [[ "${actual_commit}" != "${DTDYNAMICS_DTMATH_COMMIT}" ]]; then
    echo "ERROR: immutable dtMath checkout SHA mismatch" >&2
    exit 1
fi

if [[ ! -f "${DTDYNAMICS_DTMATH_INCLUDE_DIR}/dtMath/dtMath.h" ]]; then
    echo "ERROR: immutable dtMath checkout does not contain dtMath/dtMath.h" >&2
    exit 1
fi

echo "dtMath immutable checkout verified: ${actual_commit}"
