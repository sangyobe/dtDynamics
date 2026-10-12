#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DTDYNAMICS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

DEPS_DIR="${DTDYNAMICS_VALIDATION_DEPS_DIR:-${DTDYNAMICS_ROOT}/.validation_deps}"
MUJOCO_VERSION="3.8.0"
PINOCCHIO_VERSION="3.9.0"

if [[ -n "${DTDYNAMICS_MUJOCO_VERSION:-}" && "${DTDYNAMICS_MUJOCO_VERSION}" != "${MUJOCO_VERSION}" ]]; then
    echo "error: DTDYNAMICS_MUJOCO_VERSION is not supported; MuJoCo ${MUJOCO_VERSION} is the immutable validation dependency" >&2
    exit 2
fi
if [[ -n "${DTDYNAMICS_PINOCCHIO_VERSION:-}" && "${DTDYNAMICS_PINOCCHIO_VERSION}" != "${PINOCCHIO_VERSION}" ]]; then
    echo "error: DTDYNAMICS_PINOCCHIO_VERSION is not supported; Pinocchio ${PINOCCHIO_VERSION} is the validation dependency" >&2
    exit 2
fi

INSTALL_MUJOCO=0
INSTALL_PINOCCHIO=0
FORCE=0

usage() {
    cat <<EOF
Usage: $0 [--all] [--mujoco] [--pinocchio] [--force] [--deps-dir DIR]

Installs validation-only dependencies under the dtDynamics package by default:
  ${DEPS_DIR}

Pinned defaults:
  MuJoCo     ${MUJOCO_VERSION} (downloaded from its immutable release URL and SHA-256 verified)
  Pinocchio  ${PINOCCHIO_VERSION} (requires a caller-supplied immutable archive)

Pinocchio provisioning requires both of these environment variables:
  DTDYNAMICS_PINOCCHIO_ARCHIVE=/path/to/pinocchio-${PINOCCHIO_VERSION}.tar.gz
  DTDYNAMICS_PINOCCHIO_SHA256=REPLACE_WITH_64_HEX_SHA256

Supply both values on every invocation, including when reusing an existing prefix;
the existing prefix must have matching immutable provenance.

The Pinocchio archive must contain a top-level pinocchio-${PINOCCHIO_VERSION}/ CMake install prefix,
including headers, a Pinocchio library, pinocchioConfig.cmake, pinocchioConfigVersion.cmake,
and every CMake package dependency needed by that config.
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

verify_archive_layout() {
    local archive="$1"
    local expected_top_level="$2"

    if ! command -v python3 >/dev/null 2>&1; then
        echo "error: python3 is required to validate validation-dependency archive layout" >&2
        return 1
    fi
    python3 - "${archive}" "${expected_top_level}" <<'PY'
from pathlib import PurePosixPath
import tarfile
import sys

archive_path, expected_top_level = sys.argv[1:]


def fail(message):
    raise SystemExit(f"error: unsafe validation-dependency archive: {message}")


def parts_for_member(name):
    if not name or name.startswith("/") or "\x00" in name:
        fail(f"unsafe member name: {name!r}")
    parts = [part for part in PurePosixPath(name).parts if part not in ("", ".")]
    if not parts or any(part == ".." for part in parts):
        fail(f"unsafe member name: {name!r}")
    if parts[0] != expected_top_level:
        fail(f"member is outside {expected_top_level}/: {name!r}")
    return parts


def resolve_symlink(base, target, name):
    if not target or target.startswith("/") or "\x00" in target:
        fail(f"unsafe symlink target for {name!r}: {target!r}")
    resolved = list(base)
    for part in PurePosixPath(target).parts:
        if part in ("", "."):
            continue
        if part == "..":
            if len(resolved) <= 1:
                fail(f"symlink escapes archive root for {name!r}: {target!r}")
            resolved.pop()
        else:
            resolved.append(part)
    if not resolved or resolved[0] != expected_top_level:
        fail(f"symlink escapes archive root for {name!r}: {target!r}")


try:
    with tarfile.open(archive_path, "r:*") as archive:
        saw_top_level = False
        for member in archive.getmembers():
            parts = parts_for_member(member.name)
            saw_top_level = saw_top_level or parts == [expected_top_level]
            if member.isdir() or member.isfile():
                continue
            if member.issym():
                resolve_symlink(parts[:-1], member.linkname, member.name)
                continue
            if member.islnk():
                parts_for_member(member.linkname)
                continue
            fail(f"unsupported archive member type for {member.name!r}")
        if not saw_top_level:
            fail(f"missing top-level directory: {expected_top_level}/")
except (OSError, tarfile.TarError) as error:
    fail(str(error))
PY
}

promote_generated_prefix() {
    local candidate="$1"
    local root="$2"
    local backup="${DEPS_DIR}/.$(basename "${root}").backup.$$"
    local status

    case "${root}" in
        "${DEPS_DIR}"/*)
            ;;
        *)
            echo "error: refusing to promote non-validation path: ${root}" >&2
            return 1
            ;;
    esac
    if [[ -e "${backup}" || -L "${backup}" ]]; then
        echo "error: validation backup path already exists: ${backup}" >&2
        return 1
    fi
    if [[ -e "${root}" || -L "${root}" ]]; then
        mv "${root}" "${backup}" || return 1
    fi
    if mv "${candidate}" "${root}"; then
        if [[ -e "${backup}" || -L "${backup}" ]]; then
            if ! rm -rf "${backup}"; then
                echo "warning: installed validation prefix; cleanup of prior validation backup failed: ${backup}" >&2
            fi
        fi
        return 0
    else
        status=$?
        echo "error: failed to promote validation prefix: ${root}" >&2
        if [[ -e "${backup}" || -L "${backup}" ]]; then
            mv "${backup}" "${root}" || {
                echo "error: failed to restore prior validation prefix: ${root}" >&2
                return 1
            }
        fi
        return "${status}"
    fi
}

write_validation_provenance() {
    local prefix="$1"
    local component="$2"
    local version="$3"
    local archive_sha256="$4"
    local provenance_file="${prefix}/.dtdynamics-validation-provenance"

    if [[ -e "${provenance_file}" || -L "${provenance_file}" ]]; then
        echo "error: validation provenance file already exists: ${provenance_file}" >&2
        return 1
    fi
    printf 'schema=1\ncomponent=%s\nversion=%s\narchive_sha256=%s\n' \
        "${component}" "${version}" "${archive_sha256}" > "${provenance_file}"
}

verify_validation_provenance() {
    local prefix="$1"
    local component="$2"
    local version="$3"
    local archive_sha256="$4"
    local provenance_file="${prefix}/.dtdynamics-validation-provenance"
    local expected
    local actual

    if [[ ! -f "${provenance_file}" || -L "${provenance_file}" ]]; then
        echo "error: validation dependency provenance is missing: ${provenance_file}" >&2
        return 1
    fi
    expected="$(printf 'schema=1\ncomponent=%s\nversion=%s\narchive_sha256=%s' \
        "${component}" "${version}" "${archive_sha256}")"
    actual="$(cat "${provenance_file}")"
    if [[ "${actual}" != "${expected}" ]]; then
        echo "error: validation dependency provenance does not match the requested immutable artifact" >&2
        return 1
    fi
}

find_pinocchio_config_dir() {
    local prefix="$1"
    local libdir
    for libdir in lib lib64; do
        local config_dir="${prefix}/${libdir}/cmake/pinocchio"
        if [[ -f "${config_dir}/pinocchioConfig.cmake" &&
              -f "${config_dir}/pinocchioConfigVersion.cmake" ]]; then
            printf '%s\n' "${config_dir}"
            return 0
        fi
    done
    return 1
}

validate_pinocchio_prefix() {
    local prefix="$1"
    local probe_root="$2"
    local config_dir
    local canonical_prefix

    config_dir="$(find_pinocchio_config_dir "${prefix}")" || {
        echo "error: Pinocchio archive did not contain a CMake-consumable prefix" >&2
        return 1
    }
    if [[ ! -f "${prefix}/include/pinocchio/fwd.hpp" ]]; then
        echo "error: Pinocchio archive did not contain Pinocchio headers" >&2
        return 1
    fi
    canonical_prefix="$(cd "${prefix}" && pwd -P)"

    mkdir -p "${probe_root}/source"
    cat > "${probe_root}/source/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.12)
project(dtDynamicsPinocchioPrefixProbe LANGUAGES CXX)

if(NOT DEFINED PINOCCHIO_PREFIX OR NOT DEFINED PINOCCHIO_CONFIG_DIR OR NOT DEFINED PINOCCHIO_VERSION)
    message(FATAL_ERROR "Pinocchio prefix probe inputs are required")
endif()
get_filename_component(PINOCCHIO_PREFIX_REAL "${PINOCCHIO_PREFIX}" REALPATH)
set(CMAKE_PREFIX_PATH "${PINOCCHIO_PREFIX_REAL}")
set(CMAKE_FIND_USE_PACKAGE_ROOT_PATH FALSE)
set(CMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH FALSE)
set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH FALSE)
set(CMAKE_FIND_USE_CMAKE_SYSTEM_PATH FALSE)
set(CMAKE_FIND_USE_INSTALL_PREFIX FALSE)
set(CMAKE_FIND_USE_PACKAGE_REGISTRY FALSE)
set(CMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY FALSE)

function(require_path_within_prefix path description)
    if("${path}" MATCHES "^\\$<")
        message(FATAL_ERROR "${description} uses an unsupported generator expression")
    endif()
    if(NOT IS_ABSOLUTE "${path}")
        message(FATAL_ERROR "${description} is not an absolute installed path: ${path}")
    endif()
    if(NOT EXISTS "${path}")
        message(FATAL_ERROR "${description} does not exist: ${path}")
    endif()
    get_filename_component(resolved_path "${path}" REALPATH)
    file(RELATIVE_PATH relative_path "${PINOCCHIO_PREFIX_REAL}" "${resolved_path}")
    if(relative_path MATCHES "^\\.\\." OR IS_ABSOLUTE "${relative_path}")
        message(FATAL_ERROR "${description} escapes the Pinocchio prefix: ${resolved_path}")
    endif()
endfunction()

function(require_pinocchio_link_dependencies target property)
    get_target_property(target_links "${target}" "${property}")
    if(NOT target_links OR target_links STREQUAL "target_links-NOTFOUND")
        return()
    endif()
    foreach(target_link IN LISTS target_links)
        if(target_link STREQUAL "" OR target_link MATCHES "^::@" OR
           target_link STREQUAL "debug" OR target_link STREQUAL "optimized" OR
           target_link STREQUAL "general")
            continue()
        endif()
        if(target_link MATCHES "^\\$<LINK_ONLY:([^<>;]+)>$")
            set(target_link "${CMAKE_MATCH_1}")
        elseif(target_link MATCHES "^\\$")
            message(FATAL_ERROR "Pinocchio config exported unsupported generator-expression dependency: ${target_link}")
        endif()
        if(TARGET "${target_link}")
            require_pinocchio_target_within_prefix("${target_link}")
        elseif(IS_ABSOLUTE "${target_link}")
            require_path_within_prefix("${target_link}" "${target} ${property} direct link dependency")
        elseif(target_link MATCHES "::")
            message(FATAL_ERROR "Pinocchio config referenced an unavailable dependency target: ${target_link}")
        elseif(target_link STREQUAL "dl" OR target_link STREQUAL "m" OR
               target_link STREQUAL "pthread" OR target_link STREQUAL "rt")
            continue()
        else()
            message(FATAL_ERROR "Pinocchio config exported an unsupported raw link dependency: ${target_link}")
        endif()
    endforeach()
endfunction()

function(require_pinocchio_target_within_prefix target)
    get_property(checked_targets GLOBAL PROPERTY DTDYNAMICS_PINOCCHIO_CHECKED_TARGETS)
    list(FIND checked_targets "${target}" target_index)
    if(NOT target_index EQUAL -1)
        return()
    endif()
    set_property(GLOBAL APPEND PROPERTY DTDYNAMICS_PINOCCHIO_CHECKED_TARGETS "${target}")

    get_target_property(imported_target "${target}" IMPORTED)
    if(NOT imported_target)
        message(FATAL_ERROR "Pinocchio config exported a non-imported target: ${target}")
    endif()
    get_target_property(target_type "${target}" TYPE)
    set(location_properties
        IMPORTED_LOCATION
        IMPORTED_LOCATION_RELEASE
        IMPORTED_LOCATION_RELWITHDEBINFO
        IMPORTED_LOCATION_MINSIZEREL
        IMPORTED_LOCATION_DEBUG
        IMPORTED_IMPLIB
        IMPORTED_IMPLIB_RELEASE
        IMPORTED_IMPLIB_RELWITHDEBINFO
        IMPORTED_IMPLIB_MINSIZEREL
        IMPORTED_IMPLIB_DEBUG)
    get_target_property(target_configurations "${target}" IMPORTED_CONFIGURATIONS)
    if(target_configurations AND NOT target_configurations STREQUAL "target_configurations-NOTFOUND")
        foreach(target_configuration IN LISTS target_configurations)
            string(TOUPPER "${target_configuration}" target_configuration_upper)
            list(APPEND location_properties
                "IMPORTED_LOCATION_${target_configuration_upper}"
                "IMPORTED_IMPLIB_${target_configuration_upper}")
        endforeach()
    endif()
    list(REMOVE_DUPLICATES location_properties)
    set(target_has_library_location FALSE)
    foreach(location_property IN LISTS location_properties)
        get_target_property(target_location "${target}" "${location_property}")
        if(target_location AND NOT target_location STREQUAL "target_location-NOTFOUND")
            require_path_within_prefix("${target_location}" "${target} ${location_property}")
            set(target_has_library_location TRUE)
            if(target MATCHES "^pinocchio::")
                get_filename_component(target_library_name "${target_location}" NAME)
                if(target_library_name MATCHES "^libpinocchio([_.-].*)?\\.(so(\\.[0-9.]+)?|a)$")
                    set_property(GLOBAL PROPERTY DTDYNAMICS_PINOCCHIO_HAS_LIBRARY TRUE)
                endif()
            endif()
        endif()
    endforeach()
    if(NOT target_type STREQUAL "INTERFACE_LIBRARY" AND NOT target_has_library_location)
        message(FATAL_ERROR "Pinocchio imported library target has no installed library location: ${target}")
    endif()

    foreach(include_property INTERFACE_INCLUDE_DIRECTORIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES)
        get_target_property(target_includes "${target}" "${include_property}")
        if(target_includes AND NOT target_includes STREQUAL "target_includes-NOTFOUND")
            foreach(target_include IN LISTS target_includes)
                if(target_include MATCHES "^\\$<TARGET_PROPERTY:([^,<>;]+),(INTERFACE_INCLUDE_DIRECTORIES|INTERFACE_SYSTEM_INCLUDE_DIRECTORIES)>$")
                    set(include_property_target "${CMAKE_MATCH_1}")
                    if(NOT TARGET "${include_property_target}")
                        message(FATAL_ERROR "${target} include generator expression references an unavailable target: ${include_property_target}")
                    endif()
                    require_pinocchio_target_within_prefix("${include_property_target}")
                elseif(target_include MATCHES "^\\$<")
                    message(FATAL_ERROR "${target} include directory uses an unsupported generator expression: ${target_include}")
                else()
                    require_path_within_prefix("${target_include}" "${target} include directory")
                endif()
            endforeach()
        endif()
    endforeach()
    get_target_property(target_link_directories "${target}" INTERFACE_LINK_DIRECTORIES)
    if(target_link_directories AND NOT target_link_directories STREQUAL "target_link_directories-NOTFOUND")
        foreach(target_link_directory IN LISTS target_link_directories)
            require_path_within_prefix("${target_link_directory}" "${target} link directory")
        endforeach()
    endif()
    get_target_property(target_link_options "${target}" INTERFACE_LINK_OPTIONS)
    if(target_link_options AND NOT target_link_options STREQUAL "target_link_options-NOTFOUND")
        message(FATAL_ERROR "Pinocchio config exported unsupported interface link options: ${target}")
    endif()
    set(link_dependency_properties
        INTERFACE_LINK_LIBRARIES
        INTERFACE_LINK_LIBRARIES_DIRECT
        IMPORTED_LINK_INTERFACE_LIBRARIES
        IMPORTED_LINK_DEPENDENT_LIBRARIES)
    if(target_configurations AND NOT target_configurations STREQUAL "target_configurations-NOTFOUND")
        foreach(target_configuration IN LISTS target_configurations)
            string(TOUPPER "${target_configuration}" target_configuration_upper)
            list(APPEND link_dependency_properties
                "IMPORTED_LINK_INTERFACE_LIBRARIES_${target_configuration_upper}"
                "IMPORTED_LINK_DEPENDENT_LIBRARIES_${target_configuration_upper}")
        endforeach()
    endif()
    list(REMOVE_DUPLICATES link_dependency_properties)
    foreach(link_dependency_property IN LISTS link_dependency_properties)
        require_pinocchio_link_dependencies("${target}" "${link_dependency_property}")
    endforeach()
endfunction()

find_package(pinocchio "${PINOCCHIO_VERSION}" EXACT CONFIG REQUIRED
    PATHS "${PINOCCHIO_CONFIG_DIR}"
    NO_DEFAULT_PATH)
if(NOT TARGET pinocchio::pinocchio)
    message(FATAL_ERROR "Pinocchio config did not export pinocchio::pinocchio")
endif()
require_pinocchio_target_within_prefix(pinocchio::pinocchio)
get_property(pinocchio_has_library GLOBAL PROPERTY DTDYNAMICS_PINOCCHIO_HAS_LIBRARY)
if(NOT pinocchio_has_library)
    message(FATAL_ERROR "Pinocchio config did not export a linkable Pinocchio library")
endif()

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/main.cpp" "#include <pinocchio/fwd.hpp>\nint main() { return 0; }\n")
add_executable(dtDynamicsPinocchioPrefixProbeConsumer "${CMAKE_CURRENT_BINARY_DIR}/main.cpp")
target_link_libraries(dtDynamicsPinocchioPrefixProbeConsumer PRIVATE pinocchio::pinocchio)
CMAKE
    if ! cmake -S "${probe_root}/source" -B "${probe_root}/build" \
        -DCMAKE_BUILD_TYPE=Release \
        -DPINOCCHIO_PREFIX="${canonical_prefix}" \
        -DPINOCCHIO_CONFIG_DIR="${config_dir}" \
        -DPINOCCHIO_VERSION="${PINOCCHIO_VERSION}" >/dev/null; then
        echo "error: Pinocchio archive did not contain a CMake-consumable prefix" >&2
        return 1
    fi
    if ! cmake --build "${probe_root}/build" >/dev/null; then
        echo "error: Pinocchio archive did not contain a linkable CMake consumer prefix" >&2
        return 1
    fi
}

install_mujoco() {
    local root="${DEPS_DIR}/mujoco-${MUJOCO_VERSION}"
    local url="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz"
    local mujoco_sha256="2be88c6f92a06c3eaffdb47d3a6d3fbf159fbc057e9d272d592fb194e41fefab"

    if [[ -L "${root}" || ( -e "${root}" && ! -d "${root}" ) ]]; then
        echo "error: existing MuJoCo validation prefix is not a directory: ${root}" >&2
        return 1
    fi
    if [[ -d "${root}" && ${FORCE} -eq 0 ]]; then
        verify_validation_provenance "${root}" mujoco "${MUJOCO_VERSION}" "${mujoco_sha256}" || return 1
        echo "MuJoCo ${MUJOCO_VERSION} already exists and matches immutable provenance: ${root}"
        return
    fi

    local tmp
    tmp="$(mktemp -d "${DEPS_DIR}/.dtdynamics-mujoco.XXXXXX")"
    trap 'rm -rf "${tmp}"' RETURN

    echo "Downloading MuJoCo ${MUJOCO_VERSION}"
    download_file "${url}" "${tmp}/mujoco.tar.gz" || return 1
    verify_sha256 "${mujoco_sha256}" "${tmp}/mujoco.tar.gz" || return 1
    verify_archive_layout "${tmp}/mujoco.tar.gz" "mujoco-${MUJOCO_VERSION}" || return 1
    tar -xzf "${tmp}/mujoco.tar.gz" -C "${tmp}" || return 1

    local extracted
    extracted="$(find "${tmp}" -maxdepth 1 -type d -name "mujoco-*" | head -n 1)"
    if [[ -z "${extracted}" ]]; then
        echo "error: MuJoCo archive did not contain a mujoco-* directory" >&2
        exit 1
    fi

    write_validation_provenance "${extracted}" mujoco "${MUJOCO_VERSION}" "${mujoco_sha256}" || return 1
    promote_generated_prefix "${extracted}" "${root}" || return 1
    echo "Installed MuJoCo ${MUJOCO_VERSION}: ${root}"
    echo "CMake: -DDTDYNAMICS_MUJOCO_ROOT=${root}"
}

install_pinocchio() {
    local root="${DEPS_DIR}/pinocchio-${PINOCCHIO_VERSION}"
    local pinocchio_archive="${DTDYNAMICS_PINOCCHIO_ARCHIVE:-}"
    local pinocchio_sha256="${DTDYNAMICS_PINOCCHIO_SHA256:-}"

    if [[ -z "${pinocchio_archive}" || -z "${pinocchio_sha256}" ]]; then
        echo "error: DTDYNAMICS_PINOCCHIO_ARCHIVE and DTDYNAMICS_PINOCCHIO_SHA256 are required for immutable Pinocchio provisioning" >&2
        return 1
    fi
    if [[ ! -f "${pinocchio_archive}" ]]; then
        echo "error: immutable Pinocchio archive does not exist: ${pinocchio_archive}" >&2
        return 1
    fi
    if [[ ! "${pinocchio_sha256}" =~ ^[[:xdigit:]]{64}$ ]]; then
        echo "error: DTDYNAMICS_PINOCCHIO_SHA256 must be a 64-digit SHA-256 digest" >&2
        return 1
    fi
    pinocchio_sha256="${pinocchio_sha256,,}"

    local tmp
    local staged_archive
    tmp="$(mktemp -d "${DEPS_DIR}/.dtdynamics-pinocchio.XXXXXX")"
    trap 'rm -rf "${tmp}"' RETURN
    staged_archive="${tmp}/pinocchio.tar.gz"
    cp -- "${pinocchio_archive}" "${staged_archive}" || return 1
    verify_sha256 "${pinocchio_sha256}" "${staged_archive}" || return 1
    verify_archive_layout "${staged_archive}" "pinocchio-${PINOCCHIO_VERSION}" || return 1

    if [[ -L "${root}" || ( -e "${root}" && ! -d "${root}" ) ]]; then
        echo "error: existing Pinocchio validation prefix is not a directory: ${root}" >&2
        return 1
    fi
    if [[ -d "${root}" && ${FORCE} -eq 0 ]]; then
        validate_pinocchio_prefix "${root}" "${tmp}/existing-prefix-probe" || return 1
        verify_validation_provenance "${root}" pinocchio "${PINOCCHIO_VERSION}" "${pinocchio_sha256}" || return 1
        echo "Pinocchio ${PINOCCHIO_VERSION} already exists and matches the requested immutable archive: ${root}"
        return
    fi

    tar -xzf "${staged_archive}" -C "${tmp}" || return 1

    local extracted
    extracted="$(find "${tmp}" -maxdepth 1 -type d -name "pinocchio-${PINOCCHIO_VERSION}" | head -n 1)"
    if [[ -z "${extracted}" ]]; then
        echo "error: Pinocchio archive did not contain pinocchio-${PINOCCHIO_VERSION}" >&2
        return 1
    fi
    validate_pinocchio_prefix "${extracted}" "${tmp}/pinocchio-prefix-probe" || return 1
    write_validation_provenance "${extracted}" pinocchio "${PINOCCHIO_VERSION}" "${pinocchio_sha256}" || return 1

    promote_generated_prefix "${extracted}" "${root}" || return 1
    echo "Installed Pinocchio ${PINOCCHIO_VERSION}: ${root}"
    echo "CMake: -DDTDYNAMICS_PINOCCHIO_ROOT=${root}"
}

if [[ ${INSTALL_MUJOCO} -eq 1 ]]; then
    install_mujoco
fi

if [[ ${INSTALL_PINOCCHIO} -eq 1 ]]; then
    install_pinocchio
fi
