#!/usr/bin/env python3
"""Exercise dtDynamics package-config include precedence in isolation."""
from __future__ import annotations

import pathlib
import subprocess
import sys
import tempfile


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
TEMPLATE = REPO_ROOT / "cmake" / "dtDynamicsConfig.cmake.in"


def main() -> int:
    template = TEMPLATE.read_text(encoding="utf-8")
    with tempfile.TemporaryDirectory(prefix="dtdynamics-config-order-") as tmp:
        root = pathlib.Path(tmp)
        prefix = root / "prefix"
        config_dir = prefix / "lib" / "cmake" / "dtDynamics"
        config_dir.mkdir(parents=True)
        (prefix / "include" / "dtMath").mkdir(parents=True)
        (prefix / "include" / "dtMath_3_0_0" / "dtMath").mkdir(parents=True)
        (prefix / "include" / "dtMath" / "dtMath.h").write_text(
            "// older generic dtMath\n", encoding="utf-8"
        )
        (prefix / "include" / "dtMath_3_0_0" / "dtMath" / "dtMath.h").write_text(
            "// selected versioned dtMath\n", encoding="utf-8"
        )

        package_init = """
get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
macro(set_and_check variable path)
    set(${variable} "${path}")
endmacro()
macro(check_required_components package)
endmacro()
""".strip()
        config = template.replace("@PACKAGE_INIT@", package_init).replace(
            "@PACKAGE_CMAKE_INSTALL_INCLUDEDIR@", "${PACKAGE_PREFIX_DIR}/include"
        )
        (config_dir / "dtDynamicsConfig.cmake").write_text(config, encoding="utf-8")
        (config_dir / "dtDynamicsTargets.cmake").write_text(
            "add_library(dt::Dynamics INTERFACE IMPORTED GLOBAL)\n"
            "set_property(TARGET dt::Dynamics PROPERTY "
            "INTERFACE_INCLUDE_DIRECTORIES \"${PACKAGE_PREFIX_DIR}/include\")\n",
            encoding="utf-8",
        )

        consumer = root / "consumer"
        consumer.mkdir()
        (consumer / "CMakeLists.txt").write_text(
            "cmake_minimum_required(VERSION 3.16)\n"
            "project(dtDynamicsConfigOrder NONE)\n"
            "set(DTDYNAMICS_DTMATH_INCLUDE_DIR \"${CMAKE_CURRENT_LIST_DIR}/../prefix/include/dtMath_3_0_0\")\n"
            "find_package(dtDynamics CONFIG REQUIRED PATHS \"${CMAKE_CURRENT_LIST_DIR}/../prefix/lib/cmake/dtDynamics\" NO_DEFAULT_PATH)\n"
            "get_target_property(_includes dt::Dynamics INTERFACE_INCLUDE_DIRECTORIES)\n"
            "list(FIND _includes \"${DTDYNAMICS_DTMATH_INCLUDE_DIR}\" _selected_index)\n"
            "list(FIND _includes \"${PACKAGE_PREFIX_DIR}/include\" _generic_index)\n"
            "if(_selected_index EQUAL -1 OR _generic_index EQUAL -1 OR _selected_index GREATER _generic_index)\n"
            "  message(FATAL_ERROR \"selected dtMath include must precede generic package include: ${_includes}\")\n"
            "endif()\n",
            encoding="utf-8",
        )
        result = subprocess.run(
            ["cmake", "-S", str(consumer), "-B", str(root / "build")],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        if result.returncode != 0:
            sys.stdout.write(result.stdout)
            return result.returncode
    print("dtDynamics config include precedence: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
