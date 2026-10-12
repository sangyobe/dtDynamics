#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DTDYNAMICS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

DEPS_DIR="${DTDYNAMICS_VALIDATION_DEPS_DIR:-${DTDYNAMICS_ROOT}/.validation_deps}"
MUJOCO_VERSION="${DTDYNAMICS_MUJOCO_VERSION:-3.8.0}"
PINOCCHIO_VERSION="${DTDYNAMICS_PINOCCHIO_VERSION:-3.9.0}"

INSTALL_MUJOCO=0
INSTALL_PINOCCHIO=0
FORCE=0

usage() {
    cat <<EOF
Usage: $0 [--all] [--mujoco] [--pinocchio] [--force] [--deps-dir DIR]

Installs validation-only dependencies under the dtDynamics package by default:
  ${DEPS_DIR}

Pinned defaults:
  MuJoCo     ${MUJOCO_VERSION}
  Pinocchio  ${PINOCCHIO_VERSION}

The production dtDynamics target does not use these dependencies.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --all)
            INSTALL_MUJOCO=1
            INSTALL_PINOCCHIO=1
            ;;
        --mujoco)
            INSTALL_MUJOCO=1
            ;;
        --pinocchio)
            INSTALL_PINOCCHIO=1
            ;;
        --force)
            FORCE=1
            ;;
        --deps-dir)
            if [[ $# -lt 2 ]]; then
                echo "error: --deps-dir requires a path" >&2
                exit 2
            fi
            DEPS_DIR="$2"
            shift
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "error: unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

if [[ ${INSTALL_MUJOCO} -eq 0 && ${INSTALL_PINOCCHIO} -eq 0 ]]; then
    INSTALL_MUJOCO=1
    INSTALL_PINOCCHIO=1
fi

mkdir -p "${DEPS_DIR}"
DEPS_DIR="$(cd "${DEPS_DIR}" && pwd)"

download_file() {
    local url="$1"
    local dst="$2"

    if command -v curl >/dev/null 2>&1; then
        curl -L --fail --retry 3 -o "${dst}" "${url}"
    elif command -v wget >/dev/null 2>&1; then
        wget -O "${dst}" "${url}"
    else
        echo "error: neither curl nor wget is available" >&2
        exit 1
    fi
}

verify_sha256() {
    local expected="$1"
    local file="$2"

    if ! command -v sha256sum >/dev/null 2>&1; then
        echo "error: sha256sum is required to verify downloaded validation dependencies" >&2
        return 1
    fi
    printf '%s  %s\n' "${expected}" "${file}" | sha256sum --check --status
}

ensure_conda_tool() {
    if command -v micromamba >/dev/null 2>&1; then
        command -v micromamba
        return
    fi
    if command -v mamba >/dev/null 2>&1; then
        command -v mamba
        return
    fi
    if command -v conda >/dev/null 2>&1; then
        command -v conda
        return
    fi

    local micromamba_version="2.9.0-0"
    local micromamba_url="https://github.com/mamba-org/micromamba-releases/releases/download/${micromamba_version}/micromamba-linux-64.tar.bz2"
    local micromamba_sha256="8761c382127e6363bd9e0a2451aa3ef90d071a79133f736e2f759a3bf13040dd"
    local micromamba_root="${DEPS_DIR}/micromamba-${micromamba_version}"
    local micromamba_bin="${micromamba_root}/bin/micromamba"
    if [[ -x "${micromamba_bin}" ]]; then
        echo "${micromamba_bin}"
        return
    fi

    mkdir -p "${micromamba_root}"
    local tmp
    tmp="$(mktemp -d)"
    trap 'rm -rf "${tmp}"' RETURN

    echo "Bootstrapping private micromamba for Pinocchio installation" >&2
    download_file "${micromamba_url}" "${tmp}/micromamba.tar.bz2" || return 1
    verify_sha256 "${micromamba_sha256}" "${tmp}/micromamba.tar.bz2" || return 1
    tar -xjf "${tmp}/micromamba.tar.bz2" -C "${micromamba_root}" || return 1
    chmod 755 "${micromamba_bin}" || return 1
    if [[ ! -x "${micromamba_bin}" ]]; then
        echo "error: micromamba bootstrap did not create ${micromamba_bin}" >&2
        return 1
    fi
    echo "${micromamba_bin}"
}

remove_generated_prefix() {
    local path="$1"
    case "${path}" in
        "${DEPS_DIR}"/*)
            rm -rf "${path}"
            ;;
        *)
            echo "error: refusing to remove non-validation path: ${path}" >&2
            exit 1
            ;;
    esac
}

install_mujoco() {
    local root="${DEPS_DIR}/mujoco-${MUJOCO_VERSION}"
    local url="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz"

    if [[ -d "${root}" && ${FORCE} -eq 0 ]]; then
        echo "MuJoCo ${MUJOCO_VERSION} already exists: ${root}"
        return
    fi

    if [[ -d "${root}" ]]; then
        remove_generated_prefix "${root}"
    fi

    local tmp
    tmp="$(mktemp -d)"
    trap 'rm -rf "${tmp}"' RETURN

    echo "Downloading MuJoCo ${MUJOCO_VERSION}"
    download_file "${url}" "${tmp}/mujoco.tar.gz"
    tar -xzf "${tmp}/mujoco.tar.gz" -C "${tmp}"

    local extracted
    extracted="$(find "${tmp}" -maxdepth 1 -type d -name "mujoco-*" | head -n 1)"
    if [[ -z "${extracted}" ]]; then
        echo "error: MuJoCo archive did not contain a mujoco-* directory" >&2
        exit 1
    fi

    mv "${extracted}" "${root}"
    echo "Installed MuJoCo ${MUJOCO_VERSION}: ${root}"
    echo "CMake: -DDTDYNAMICS_MUJOCO_ROOT=${root}"
}

install_pinocchio() {
    local root="${DEPS_DIR}/pinocchio-${PINOCCHIO_VERSION}"

    if [[ -d "${root}" && ${FORCE} -eq 0 ]]; then
        echo "Pinocchio ${PINOCCHIO_VERSION} already exists: ${root}"
        return
    fi

    if [[ -d "${root}" ]]; then
        remove_generated_prefix "${root}"
    fi

    local conda_tool
    conda_tool="$(ensure_conda_tool)" || {
        echo "error: could not provision a conda-compatible tool for Pinocchio" >&2
        return 1
    }
    MAMBA_ROOT_PREFIX="${DEPS_DIR}/micromamba-root" \
        "${conda_tool}" create -y -p "${root}" -c conda-forge "pinocchio=${PINOCCHIO_VERSION}"

    echo "Installed Pinocchio ${PINOCCHIO_VERSION}: ${root}"
    echo "CMake: -DDTDYNAMICS_PINOCCHIO_ROOT=${root}"
}

if [[ ${INSTALL_MUJOCO} -eq 1 ]]; then
    install_mujoco
fi

if [[ ${INSTALL_PINOCCHIO} -eq 1 ]]; then
    install_pinocchio
fi
