#!/usr/bin/env bash
set -euo pipefail

echo "=== Build Environment ==="

if [[ -n "${ARTF_INSTALL_DIR:-}" && -f "${ARTF_INSTALL_DIR}/versions.txt" ]]; then
    cat "${ARTF_INSTALL_DIR}/versions.txt"
else
    echo "INFO: ${ARTF_INSTALL_DIR:-ARTF_INSTALL_DIR}/versions.txt 없음 - 계속 진행"
fi

echo "========================="

dtmath_cmake_args=()
if [[ -n "${DTDYNAMICS_DTMATH_INCLUDE_DIR:-}" ]]; then
    dtmath_cmake_args+=("-DDTDYNAMICS_DTMATH_INCLUDE_DIR=${DTDYNAMICS_DTMATH_INCLUDE_DIR}")
fi

cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DDTDYNAMICS_BUILD_TEST=OFF \
    -DDTDYNAMICS_BUILD_BENCHMARK=OFF \
    -DDTDYNAMICS_BUILD_REFERENCE_BENCHMARK=OFF \
    -DBUILD_DTDYNAMICS_MUJOCO_TEST=OFF \
    -DBUILD_DTDYNAMICS_PINOCCHIO_TEST=OFF \
    "${dtmath_cmake_args[@]}"

cmake --build build -j"$(nproc)"

echo "=== Build complete ==="
