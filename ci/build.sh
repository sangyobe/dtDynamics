#!/usr/bin/env bash
set -euo pipefail

echo "=== Build Environment ==="

if [[ -n "${ARTF_INSTALL_DIR:-}" && -f "${ARTF_INSTALL_DIR}/versions.txt" ]]; then
    cat "${ARTF_INSTALL_DIR}/versions.txt"
else
    echo "INFO: ${ARTF_INSTALL_DIR:-ARTF_INSTALL_DIR}/versions.txt 없음 - 계속 진행"
fi

echo "========================="

cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DDTDYNAMICS_BUILD_TEST=ON \
    -DDTDYNAMICS_BUILD_BENCHMARK=OFF

cmake --build build -j"$(nproc)"

echo "=== Build complete ==="
