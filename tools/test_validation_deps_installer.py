#!/usr/bin/env python3
"""Behavioral contracts for immutable validation-dependency provisioning."""

from __future__ import annotations

import hashlib
import io
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile


ROOT = Path(__file__).resolve().parents[1]
INSTALLER = ROOT / "tools" / "install_validation_deps.sh"
MUJOCO_SHA256 = "2be88c6f92a06c3eaffdb47d3a6d3fbf159fbc057e9d272d592fb194e41fefab"
PINOCCHIO_VERSION = "3.9.0"


def write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def write_archive(
    path: Path,
    directory: str,
    *,
    cmake_ready: bool = False,
    libdir: str = "lib",
    target_location: str | None = None,
    include_legacy_header: bool = True,
    external_dependency: bool = False,
    interface_only: bool = False,
    foreign_target_location: str | None = None,
    direct_link_target_location: str | None = None,
    legacy_link_interface_location: str | None = None,
    legacy_dependent_library_location: str | None = None,
    configured_legacy_link_interface_location: str | None = None,
    target_property_include_reference: bool = False,
    payload_marker: str = "replacement payload",
) -> None:
    archive_root = path.parent / "archive-root"
    prefix = archive_root / directory
    write_text(prefix / "UNVERIFIED_PAYLOAD", "replacement payload\n")
    write_text(prefix / "PAYLOAD_MARKER", f"{payload_marker}\n")
    if cmake_ready:
        write_text(prefix / "include/pinocchio/fwd.hpp", "#pragma once\n")
        if include_legacy_header:
            write_text(prefix / "include/pinocchio/pinocchio.hpp", "#pragma once\n")
        config_dir = prefix / f"{libdir}/cmake/pinocchio"
        if interface_only:
            config = (
                "set(pinocchio_VERSION \"3.9.0\")\n"
                "add_library(pinocchio::pinocchio INTERFACE IMPORTED)\n"
                "set_target_properties(pinocchio::pinocchio PROPERTIES\n"
                "  INTERFACE_INCLUDE_DIRECTORIES \"${CMAKE_CURRENT_LIST_DIR}/../../../include\")\n"
            )
        else:
            source = prefix / "fixture.cpp"
            library = prefix / libdir / "libpinocchio_default.so"
            library.parent.mkdir(parents=True, exist_ok=True)
            write_text(source, 'extern "C" int dtdynamics_pinocchio_fixture() { return 0; }\n')
            subprocess.run(
                ["c++", "-shared", "-fPIC", str(source), "-o", str(library)],
                check=True,
                capture_output=True,
                text=True,
            )
            location = target_location or "${CMAKE_CURRENT_LIST_DIR}/../../libpinocchio_default.so"
            dependency = "find_package(auxiliary CONFIG REQUIRED)\n" if external_dependency else ""
            foreign_target = ""
            foreign_link = ""
            direct_link_target = ""
            legacy_link_interface = ""
            legacy_dependent_library = ""
            configured_legacy_link_interface = ""
            target_property_include_target = ""
            default_include_directories = "${CMAKE_CURRENT_LIST_DIR}/../../../include"
            if target_property_include_reference:
                target_property_include_target = (
                    "if(NOT TARGET pinocchio::pinocchio_headers)\n"
                    "  add_library(pinocchio::pinocchio_headers INTERFACE IMPORTED)\n"
                    "  set_target_properties(pinocchio::pinocchio_headers PROPERTIES\n"
                    "    INTERFACE_INCLUDE_DIRECTORIES \"${CMAKE_CURRENT_LIST_DIR}/../../../include\")\n"
                    "endif()\n"
                )
                default_include_directories = (
                    "${CMAKE_CURRENT_LIST_DIR}/../../../include;"
                    "$<TARGET_PROPERTY:pinocchio::pinocchio_headers,INTERFACE_INCLUDE_DIRECTORIES>"
                )
            auxiliary_target_location = foreign_target_location or direct_link_target_location
            if auxiliary_target_location is not None:
                foreign_target = (
                    "add_library(auxiliary::foreign SHARED IMPORTED)\n"
                    "set_target_properties(auxiliary::foreign PROPERTIES\n"
                    f"  IMPORTED_LOCATION \"{auxiliary_target_location}\")\n"
                )
            if foreign_target_location is not None:
                foreign_link = ";auxiliary::foreign"
            if direct_link_target_location is not None:
                direct_link_target = (
                    "    INTERFACE_LINK_LIBRARIES_DIRECT \"auxiliary::foreign\"\n"
                )
            if legacy_link_interface_location is not None:
                legacy_link_interface = (
                    f"    IMPORTED_LINK_INTERFACE_LIBRARIES \"{legacy_link_interface_location}\"\n"
                )
            if configured_legacy_link_interface_location is not None:
                configured_legacy_link_interface = (
                    "    IMPORTED_CONFIGURATIONS \"RELEASE\"\n"
                    f"    IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE \"{configured_legacy_link_interface_location}\"\n"
                )
            if legacy_dependent_library_location is not None:
                legacy_dependent_library = (
                    f"    IMPORTED_LINK_DEPENDENT_LIBRARIES \"{legacy_dependent_library_location}\"\n"
                )
            config = (
                "set(pinocchio_VERSION \"3.9.0\")\n"
                f"{dependency}"
                f"{foreign_target}"
                f"{target_property_include_target}"
                "if(NOT TARGET pinocchio::pinocchio_default)\n"
                "  add_library(pinocchio::pinocchio_default SHARED IMPORTED)\n"
                "  set_target_properties(pinocchio::pinocchio_default PROPERTIES\n"
                f"    IMPORTED_LOCATION \"{location}\"\n"
                f"{legacy_link_interface}"
                f"{configured_legacy_link_interface}"
                f"{legacy_dependent_library}"
                f"    INTERFACE_INCLUDE_DIRECTORIES \"{default_include_directories}\")\n"
                "endif()\n"
                "if(NOT TARGET pinocchio::pinocchio)\n"
                "  add_library(pinocchio::pinocchio INTERFACE IMPORTED)\n"
                "  set_target_properties(pinocchio::pinocchio PROPERTIES\n"
                f"    INTERFACE_LINK_LIBRARIES \"pinocchio::pinocchio_default{foreign_link}\"\n"
                f"{direct_link_target}"
                "    INTERFACE_INCLUDE_DIRECTORIES \"${CMAKE_CURRENT_LIST_DIR}/../../../include\")\n"
                "endif()\n"
            )
        write_text(config_dir / "pinocchioConfig.cmake", config)
        write_text(
            config_dir / "pinocchioConfigVersion.cmake",
            "set(PACKAGE_VERSION \"3.9.0\")\n"
            "if(PACKAGE_FIND_VERSION VERSION_EQUAL PACKAGE_VERSION)\n"
            "  set(PACKAGE_VERSION_EXACT TRUE)\n"
            "  set(PACKAGE_VERSION_COMPATIBLE TRUE)\n"
            "elseif(PACKAGE_FIND_VERSION VERSION_LESS PACKAGE_VERSION)\n"
            "  set(PACKAGE_VERSION_COMPATIBLE TRUE)\n"
            "endif()\n",
        )
    with tarfile.open(path, "w:gz") as archive:
        archive.add(prefix, arcname=directory)


def write_executable(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")
    path.chmod(0o755)


def run_installer(root: Path, arguments: list[str], environment: dict[str, str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["bash", str(INSTALLER), *arguments, "--deps-dir", str(root / "deps")],
        cwd=root,
        env={**os.environ, **environment},
        text=True,
        capture_output=True,
        check=False,
    )


def fake_download_tools(root: Path, archive: Path) -> dict[str, str]:
    bin_dir = root / "bin"
    bin_dir.mkdir(exist_ok=True)
    write_executable(
        bin_dir / "curl",
        "#!/usr/bin/env sh\n"
        "while [ $# -gt 0 ]; do\n"
        "  if [ \"$1\" = \"-o\" ] || [ \"$1\" = \"--output\" ]; then output=$2; shift 2; continue; fi\n"
        "  shift\n"
        "done\n"
        "cp \"${FAKE_ARCHIVE}\" \"${output}\"\n",
    )
    return {
        "PATH": f"{bin_dir}:{os.environ['PATH']}",
        "FAKE_ARCHIVE": str(archive),
    }


def failing_mv_environment(root: Path, failure_number: int) -> dict[str, str]:
    bin_dir = root / "mv-bin"
    bin_dir.mkdir()
    counter = root / "mv-count"
    write_executable(
        bin_dir / "mv",
        "#!/usr/bin/env sh\n"
        "count=0\n"
        "if [ -f \"${FAKE_MV_COUNTER}\" ]; then count=$(cat \"${FAKE_MV_COUNTER}\"); fi\n"
        "count=$((count + 1))\n"
        "printf '%s' \"${count}\" > \"${FAKE_MV_COUNTER}\"\n"
        "if [ \"${count}\" -eq \"${FAKE_MV_FAIL_ON}\" ]; then exit 86; fi\n"
        "exec /bin/mv \"$@\"\n",
    )
    return {
        "PATH": f"{bin_dir}:{os.environ['PATH']}",
        "FAKE_MV_COUNTER": str(counter),
        "FAKE_MV_FAIL_ON": str(failure_number),
    }


def failing_backup_cleanup_environment(root: Path) -> dict[str, str]:
    bin_dir = root / "rm-bin"
    bin_dir.mkdir()
    counter = root / "backup-cleanup-mv-count"
    write_executable(
        bin_dir / "mv",
        "#!/usr/bin/env sh\n"
        "count=0\n"
        "if [ -f \"${FAKE_MV_COUNTER}\" ]; then count=$(cat \"${FAKE_MV_COUNTER}\"); fi\n"
        "count=$((count + 1))\n"
        "printf '%s' \"${count}\" > \"${FAKE_MV_COUNTER}\"\n"
        "if [ \"${count}\" -ge 3 ]; then exit 88; fi\n"
        "exec /bin/mv \"$@\"\n",
    )
    write_executable(
        bin_dir / "rm",
        "#!/usr/bin/env sh\n"
        "for argument in \"$@\"; do\n"
        "  case \"${argument}\" in\n"
        "    */.pinocchio-3.9.0.backup.*) exit 87 ;;\n"
        "  esac\n"
        "done\n"
        "exec /bin/rm \"$@\"\n",
    )
    return {
        "PATH": f"{bin_dir}:{os.environ['PATH']}",
        "FAKE_MV_COUNTER": str(counter),
    }


def swap_archive_before_tar_extract_environment(
    root: Path,
    archive_to_swap: Path,
    replacement_archive: Path,
) -> dict[str, str]:
    bin_dir = root / "tar-bin"
    bin_dir.mkdir()
    write_executable(
        bin_dir / "tar",
        "#!/usr/bin/env sh\n"
        "if [ \"${1:-}\" = \"-xzf\" ]; then\n"
        "  cp \"${FAKE_REPLACEMENT_ARCHIVE}\" \"${FAKE_ARCHIVE_TO_SWAP}\"\n"
        "fi\n"
        "exec /bin/tar \"$@\"\n",
    )
    return {
        "PATH": f"{bin_dir}:{os.environ['PATH']}",
        "FAKE_ARCHIVE_TO_SWAP": str(archive_to_swap),
        "FAKE_REPLACEMENT_ARCHIVE": str(replacement_archive),
    }


def append_pax_nul_member(archive: Path, directory: str) -> None:
    rewritten = archive.with_suffix(".nul.tar.gz")
    with tarfile.open(archive, "r:gz") as source, tarfile.open(
        rewritten, "w:gz", format=tarfile.PAX_FORMAT
    ) as target:
        for member in source.getmembers():
            if member.isfile():
                extracted = source.extractfile(member)
                assert extracted is not None
                target.addfile(member, extracted)
            else:
                target.addfile(member)
        unsafe = tarfile.TarInfo(f"{directory}/safe-name")
        unsafe.pax_headers = {"path": f"{directory}/safe\x00name"}
        payload = b"unsafe\n"
        unsafe.size = len(payload)
        target.addfile(unsafe, io.BytesIO(payload))
    rewritten.replace(archive)


def compile_shared_library(path: Path) -> None:
    source = path.with_suffix(".cpp")
    write_text(source, 'extern "C" int dtdynamics_foreign_fixture() { return 0; }\n')
    path.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        ["c++", "-shared", "-fPIC", str(source), "-o", str(path)],
        check=True,
        capture_output=True,
        text=True,
    )


def pinocchio_environment(root: Path, archive: Path) -> dict[str, str]:
    return {
        **fake_download_tools(root, archive),
        "DTDYNAMICS_PINOCCHIO_ARCHIVE": str(archive),
        "DTDYNAMICS_PINOCCHIO_SHA256": hashlib.sha256(archive.read_bytes()).hexdigest(),
    }


def extract_fixture_archive(archive: Path, destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    with tarfile.open(archive, "r:gz") as tar:
        tar.extractall(destination)


def replace_archive_member_with_symlink(archive: Path, member_name: str, link_name: str) -> None:
    rewritten = archive.with_suffix(".rewritten.tar.gz")
    with tarfile.open(archive, "r:gz") as source, tarfile.open(rewritten, "w:gz") as target:
        for member in source.getmembers():
            if member.name == member_name:
                replacement = tarfile.TarInfo(member_name)
                replacement.type = tarfile.SYMTYPE
                replacement.linkname = link_name
                replacement.mode = 0o777
                target.addfile(replacement)
            elif member.isfile():
                extracted = source.extractfile(member)
                assert extracted is not None
                target.addfile(member, extracted)
            else:
                target.addfile(member)
    rewritten.replace(archive)


def assert_replacement_mujoco_archive_is_rejected() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "replacement-mujoco.tar.gz"
        write_archive(archive, "mujoco-3.8.0")
        result = run_installer(root, ["--mujoco", "--force"], fake_download_tools(root, archive))
        installed_payload = root / "deps/mujoco-3.8.0/UNVERIFIED_PAYLOAD"
        assert result.returncode != 0, result.stdout + result.stderr
        assert not installed_payload.exists(), installed_payload


def assert_mujoco_version_override_cannot_escape_deps_dir() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "replacement-mujoco.tar.gz"
        write_archive(archive, "mujoco-3.8.0")
        (root / "deps/mujoco-").mkdir(parents=True)
        sentinel = root / "victim/SENTINEL"
        write_text(sentinel, "must remain\n")
        result = run_installer(
            root,
            ["--mujoco", "--force"],
            {
                **fake_download_tools(root, archive),
                "DTDYNAMICS_MUJOCO_VERSION": "/../../victim",
            },
        )
        assert result.returncode != 0, result.stdout + result.stderr
        assert sentinel.is_file(), result.stdout + result.stderr


def assert_mujoco_checksum_failure_preserves_existing_prefix() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "replacement-mujoco.tar.gz"
        write_archive(archive, "mujoco-3.8.0")
        sentinel = root / "deps/mujoco-3.8.0/SENTINEL"
        write_text(sentinel, "must remain\n")
        result = run_installer(root, ["--mujoco", "--force"], fake_download_tools(root, archive))
        assert result.returncode != 0, result.stdout + result.stderr
        assert sentinel.is_file(), result.stdout + result.stderr


def assert_existing_mujoco_prefix_requires_matching_provenance() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        write_text(root / "deps/mujoco-3.8.0/SENTINEL", "unprovenanced\n")
        result = run_installer(root, ["--mujoco"], {})
        assert result.returncode != 0, result.stdout + result.stderr
        assert "provenance" in result.stderr


def assert_pinocchio_requires_verified_archive() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "replacement-pinocchio.tar.gz"
        write_archive(archive, f"pinocchio-{PINOCCHIO_VERSION}")
        environment = {
            **fake_download_tools(root, archive),
            "DTDYNAMICS_PINOCCHIO_ARCHIVE": str(archive),
            "DTDYNAMICS_PINOCCHIO_SHA256": "0" * 64,
        }
        result = run_installer(root, ["--pinocchio", "--force"], environment)
        installed_payload = root / f"deps/pinocchio-{PINOCCHIO_VERSION}/UNVERIFIED_PAYLOAD"
        assert result.returncode != 0, result.stdout + result.stderr
        assert not installed_payload.exists(), installed_payload


def assert_pinocchio_requires_archive_inputs() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        result = run_installer(root, ["--pinocchio", "--force"], {})
        assert result.returncode != 0, result.stdout + result.stderr
        assert "DTDYNAMICS_PINOCCHIO_ARCHIVE" in result.stderr


def assert_existing_pinocchio_prefix_requires_verified_archive_inputs() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        write_text(root / f"deps/pinocchio-{PINOCCHIO_VERSION}/SENTINEL", "unprovenanced\n")
        result = run_installer(root, ["--pinocchio"], {})
        assert result.returncode != 0, result.stdout + result.stderr
        assert "DTDYNAMICS_PINOCCHIO_ARCHIVE" in result.stderr


def assert_existing_pinocchio_prefix_requires_matching_provenance() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "pinned-pinocchio.tar.gz"
        write_archive(archive, f"pinocchio-{PINOCCHIO_VERSION}", cmake_ready=True)
        extract_fixture_archive(archive, root / "deps")
        result = run_installer(root, ["--pinocchio"], pinocchio_environment(root, archive))
        assert result.returncode != 0, result.stdout + result.stderr
        assert "provenance" in result.stderr


def assert_non_cmake_pinocchio_prefix_is_rejected() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "incomplete-pinocchio.tar.gz"
        write_archive(archive, f"pinocchio-{PINOCCHIO_VERSION}")
        result = run_installer(root, ["--pinocchio", "--force"], pinocchio_environment(root, archive))
        installed_payload = root / f"deps/pinocchio-{PINOCCHIO_VERSION}/UNVERIFIED_PAYLOAD"
        assert result.returncode != 0, result.stdout + result.stderr
        assert "CMake-consumable" in result.stderr
        assert not installed_payload.exists(), installed_payload


def assert_interface_only_pinocchio_prefix_is_rejected() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "interface-only-pinocchio.tar.gz"
        write_archive(
            archive,
            f"pinocchio-{PINOCCHIO_VERSION}",
            cmake_ready=True,
            interface_only=True,
        )
        result = run_installer(root, ["--pinocchio", "--force"], pinocchio_environment(root, archive))
        assert result.returncode != 0, result.stdout + result.stderr
        assert not (root / f"deps/pinocchio-{PINOCCHIO_VERSION}").exists()


def assert_pinocchio_probe_rejects_foreign_target_dependency() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "foreign-target-pinocchio.tar.gz"
        foreign_library = root / "foreign/libforeign.so"
        compile_shared_library(foreign_library)
        write_archive(
            archive,
            f"pinocchio-{PINOCCHIO_VERSION}",
            cmake_ready=True,
            foreign_target_location=str(foreign_library),
        )
        result = run_installer(root, ["--pinocchio", "--force"], pinocchio_environment(root, archive))
        assert result.returncode != 0, result.stdout + result.stderr
        assert not (root / f"deps/pinocchio-{PINOCCHIO_VERSION}").exists()


def assert_pinocchio_probe_rejects_foreign_direct_link_dependency() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "foreign-direct-link-pinocchio.tar.gz"
        foreign_library = root / "foreign/libforeign.so"
        compile_shared_library(foreign_library)
        write_archive(
            archive,
            f"pinocchio-{PINOCCHIO_VERSION}",
            cmake_ready=True,
            direct_link_target_location=str(foreign_library),
        )
        result = run_installer(root, ["--pinocchio", "--force"], pinocchio_environment(root, archive))
        assert result.returncode != 0, result.stdout + result.stderr
        assert not (root / f"deps/pinocchio-{PINOCCHIO_VERSION}").exists()


def assert_pinocchio_probe_rejects_foreign_legacy_link_interface() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "foreign-legacy-link-pinocchio.tar.gz"
        foreign_library = root / "foreign/libforeign.so"
        compile_shared_library(foreign_library)
        write_archive(
            archive,
            f"pinocchio-{PINOCCHIO_VERSION}",
            cmake_ready=True,
            legacy_link_interface_location=str(foreign_library),
        )
        result = run_installer(root, ["--pinocchio", "--force"], pinocchio_environment(root, archive))
        assert result.returncode != 0, result.stdout + result.stderr
        assert not (root / f"deps/pinocchio-{PINOCCHIO_VERSION}").exists()


def assert_pinocchio_probe_rejects_foreign_configured_legacy_link_interface() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "foreign-configured-legacy-link-pinocchio.tar.gz"
        foreign_library = root / "foreign/libforeign.so"
        compile_shared_library(foreign_library)
        write_archive(
            archive,
            f"pinocchio-{PINOCCHIO_VERSION}",
            cmake_ready=True,
            configured_legacy_link_interface_location=str(foreign_library),
        )
        result = run_installer(root, ["--pinocchio", "--force"], pinocchio_environment(root, archive))
        assert result.returncode != 0, result.stdout + result.stderr
        assert not (root / f"deps/pinocchio-{PINOCCHIO_VERSION}").exists()


def assert_pinocchio_probe_rejects_foreign_legacy_dependent_library() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "foreign-legacy-dependent-pinocchio.tar.gz"
        foreign_library = root / "foreign/libforeign.so"
        compile_shared_library(foreign_library)
        write_archive(
            archive,
            f"pinocchio-{PINOCCHIO_VERSION}",
            cmake_ready=True,
            legacy_dependent_library_location=str(foreign_library),
        )
        result = run_installer(root, ["--pinocchio", "--force"], pinocchio_environment(root, archive))
        assert result.returncode != 0, result.stdout + result.stderr
        assert not (root / f"deps/pinocchio-{PINOCCHIO_VERSION}").exists()


def assert_pinocchio_archive_rejects_pax_nul_member() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "nul-member-pinocchio.tar.gz"
        directory = f"pinocchio-{PINOCCHIO_VERSION}"
        write_archive(archive, directory, cmake_ready=True)
        append_pax_nul_member(archive, directory)
        result = run_installer(root, ["--pinocchio", "--force"], pinocchio_environment(root, archive))
        assert result.returncode != 0, result.stdout + result.stderr
        assert not (root / f"deps/pinocchio-{PINOCCHIO_VERSION}").exists()


def assert_pinocchio_verified_archive_is_not_swapped_before_extraction() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "pinned-pinocchio.tar.gz"
        replacement = root / "replacement-pinocchio.tar.gz"
        write_archive(
            archive,
            f"pinocchio-{PINOCCHIO_VERSION}",
            cmake_ready=True,
            payload_marker="GOOD_BYTES",
        )
        write_archive(
            replacement,
            f"pinocchio-{PINOCCHIO_VERSION}",
            cmake_ready=True,
            payload_marker="REPLACEMENT_BYTES",
        )
        result = run_installer(
            root,
            ["--pinocchio", "--force"],
            {
                **pinocchio_environment(root, archive),
                **swap_archive_before_tar_extract_environment(root, archive, replacement),
            },
        )
        marker = root / f"deps/pinocchio-{PINOCCHIO_VERSION}/PAYLOAD_MARKER"
        assert result.returncode == 0, result.stdout + result.stderr
        assert marker.read_text(encoding="utf-8") == "GOOD_BYTES\n"


def assert_pinocchio_probe_accepts_prefix_target_property_include_reference() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "target-property-include-pinocchio.tar.gz"
        write_archive(
            archive,
            f"pinocchio-{PINOCCHIO_VERSION}",
            cmake_ready=True,
            target_property_include_reference=True,
        )
        result = run_installer(root, ["--pinocchio", "--force"], pinocchio_environment(root, archive))
        assert result.returncode == 0, result.stdout + result.stderr
        assert (root / f"deps/pinocchio-{PINOCCHIO_VERSION}").is_dir()


def assert_verified_cmake_prefix_archive_installs() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "pinned-pinocchio.tar.gz"
        write_archive(archive, f"pinocchio-{PINOCCHIO_VERSION}", cmake_ready=True)
        result = run_installer(root, ["--pinocchio", "--force"], pinocchio_environment(root, archive))
        prefix = root / f"deps/pinocchio-{PINOCCHIO_VERSION}"
        digest = hashlib.sha256(archive.read_bytes()).hexdigest()
        assert result.returncode == 0, result.stdout + result.stderr
        assert (prefix / "include/pinocchio/fwd.hpp").is_file()
        assert (prefix / "lib/libpinocchio_default.so").is_file()
        assert (prefix / "lib/cmake/pinocchio/pinocchioConfig.cmake").is_file()
        assert (prefix / ".dtdynamics-validation-provenance").read_text(encoding="utf-8") == (
            "schema=1\n"
            "component=pinocchio\n"
            f"version={PINOCCHIO_VERSION}\n"
            f"archive_sha256={digest}\n"
        )
        recheck = run_installer(root, ["--pinocchio"], pinocchio_environment(root, archive))
        assert recheck.returncode == 0, recheck.stdout + recheck.stderr


def assert_cmake_prefix_in_lib64_installs() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "pinned-pinocchio-lib64.tar.gz"
        write_archive(archive, f"pinocchio-{PINOCCHIO_VERSION}", cmake_ready=True, libdir="lib64")
        result = run_installer(root, ["--pinocchio", "--force"], pinocchio_environment(root, archive))
        prefix = root / f"deps/pinocchio-{PINOCCHIO_VERSION}"
        assert result.returncode == 0, result.stdout + result.stderr
        assert (prefix / "lib64/cmake/pinocchio/pinocchioConfig.cmake").is_file()


def assert_fwd_header_only_pinocchio_prefix_installs() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "pinned-pinocchio-fwd-only.tar.gz"
        write_archive(
            archive,
            f"pinocchio-{PINOCCHIO_VERSION}",
            cmake_ready=True,
            include_legacy_header=False,
        )
        result = run_installer(root, ["--pinocchio", "--force"], pinocchio_environment(root, archive))
        assert result.returncode == 0, result.stdout + result.stderr


def assert_pinocchio_probe_rejects_external_dependency_escape() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "external-dependency-pinocchio.tar.gz"
        write_archive(
            archive,
            f"pinocchio-{PINOCCHIO_VERSION}",
            cmake_ready=True,
            external_dependency=True,
        )
        external_config = root / "external/lib/cmake/auxiliary/auxiliaryConfig.cmake"
        write_text(external_config, "set(auxiliary_FOUND TRUE)\n")
        result = run_installer(
            root,
            ["--pinocchio", "--force"],
            {
                **pinocchio_environment(root, archive),
                "CMAKE_PREFIX_PATH": str(root / "external"),
            },
        )
        assert result.returncode != 0, result.stdout + result.stderr
        assert not (root / f"deps/pinocchio-{PINOCCHIO_VERSION}").exists()


def assert_pinocchio_archive_rejects_escaping_header_symlink() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "escaping-header-pinocchio.tar.gz"
        outside_header = root / "outside-header.hpp"
        write_text(outside_header, "#pragma once\n")
        write_archive(archive, f"pinocchio-{PINOCCHIO_VERSION}", cmake_ready=True)
        replace_archive_member_with_symlink(
            archive,
            f"pinocchio-{PINOCCHIO_VERSION}/include/pinocchio/fwd.hpp",
            str(outside_header),
        )
        result = run_installer(root, ["--pinocchio", "--force"], pinocchio_environment(root, archive))
        assert result.returncode != 0, result.stdout + result.stderr
        assert not (root / f"deps/pinocchio-{PINOCCHIO_VERSION}").exists()


def assert_unusable_exported_pinocchio_target_is_rejected() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "outside-target-pinocchio.tar.gz"
        write_archive(
            archive,
            f"pinocchio-{PINOCCHIO_VERSION}",
            cmake_ready=True,
            target_location="/definitely/not/in/the/archive/libpinocchio_default.so",
        )
        result = run_installer(root, ["--pinocchio", "--force"], pinocchio_environment(root, archive))
        assert result.returncode != 0, result.stdout + result.stderr
        assert not (root / f"deps/pinocchio-{PINOCCHIO_VERSION}").exists()


def assert_pinocchio_promotion_failure_restores_existing_prefix() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "pinned-pinocchio.tar.gz"
        write_archive(archive, f"pinocchio-{PINOCCHIO_VERSION}", cmake_ready=True)
        extract_fixture_archive(archive, root / "deps")
        sentinel = root / f"deps/pinocchio-{PINOCCHIO_VERSION}/SENTINEL"
        write_text(sentinel, "must remain\n")
        result = run_installer(
            root,
            ["--pinocchio", "--force"],
            {
                **pinocchio_environment(root, archive),
                **failing_mv_environment(root, failure_number=2),
            },
        )
        assert result.returncode != 0, result.stdout + result.stderr
        assert sentinel.is_file(), result.stdout + result.stderr


def assert_pinocchio_backup_cleanup_failure_commits_new_prefix() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = root / "pinned-pinocchio.tar.gz"
        write_archive(
            archive,
            f"pinocchio-{PINOCCHIO_VERSION}",
            cmake_ready=True,
            payload_marker="NEW_BYTES",
        )
        extract_fixture_archive(archive, root / "deps")
        sentinel = root / f"deps/pinocchio-{PINOCCHIO_VERSION}/SENTINEL"
        write_text(sentinel, "must remain in retained backup\n")
        result = run_installer(
            root,
            ["--pinocchio", "--force"],
            {
                **pinocchio_environment(root, archive),
                **failing_backup_cleanup_environment(root),
            },
        )
        prefix = root / f"deps/pinocchio-{PINOCCHIO_VERSION}"
        backups = list((root / "deps").glob(".pinocchio-3.9.0.backup.*"))
        assert result.returncode == 0, result.stdout + result.stderr
        assert (prefix / "PAYLOAD_MARKER").read_text(encoding="utf-8") == "NEW_BYTES\n"
        assert not sentinel.exists(), result.stdout + result.stderr
        assert len(backups) == 1, backups
        assert (backups[0] / "SENTINEL").is_file()
        assert (root / "backup-cleanup-mv-count").read_text(encoding="utf-8") == "2"
        assert "cleanup of prior validation backup failed" in result.stderr


def assert_readme_pinocchio_command_is_shell_safe() -> None:
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    command_lines = (
        "DTDYNAMICS_PINOCCHIO_ARCHIVE=/path/to/pinocchio-3.9.0.tar.gz \\\n"
        "DTDYNAMICS_PINOCCHIO_SHA256=REPLACE_WITH_64_HEX_SHA256 \\\n"
        "tools/install_validation_deps.sh --all"
    )
    assert command_lines in readme
    assert "DTDYNAMICS_PINOCCHIO_SHA256=<64-hex-SHA-256>" not in readme

    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        script = root / "tools/install_validation_deps.sh"
        script.parent.mkdir()
        shutil.copy2(INSTALLER, script)
        before = script.read_bytes()
        bin_dir = root / "bin"
        bin_dir.mkdir()
        marker = root / "curl-invoked"
        write_executable(
            bin_dir / "curl",
            "#!/usr/bin/env sh\n: > \"${FAKE_CURL_MARKER}\"\nexit 97\n",
        )
        result = subprocess.run(
            ["bash", "-c", command_lines],
            cwd=root,
            env={
                **os.environ,
                "PATH": f"{bin_dir}:{os.environ['PATH']}",
                "FAKE_CURL_MARKER": str(marker),
                "DTDYNAMICS_VALIDATION_DEPS_DIR": str(root / "validation-deps"),
            },
            text=True,
            capture_output=True,
            check=False,
        )
        assert result.returncode != 0
        assert marker.is_file()
        assert script.read_bytes() == before


def main() -> int:
    installer = INSTALLER.read_text(encoding="utf-8")
    required = (
        'mujoco_sha256="2be88c6f92a06c3eaffdb47d3a6d3fbf159fbc057e9d272d592fb194e41fefab"',
        'verify_sha256 "${mujoco_sha256}" "${tmp}/mujoco.tar.gz"',
        'DTDYNAMICS_PINOCCHIO_ARCHIVE',
        'DTDYNAMICS_PINOCCHIO_SHA256',
        'staged_archive="${tmp}/pinocchio.tar.gz"',
        'cp -- "${pinocchio_archive}" "${staged_archive}"',
        'verify_sha256 "${pinocchio_sha256}" "${staged_archive}"',
        'find_package(pinocchio "${PINOCCHIO_VERSION}" EXACT CONFIG REQUIRED',
    )
    for token in required:
        assert token in installer, f"validation dependency installer is not immutable: {token}"
    assert " -c conda-forge " not in installer, "Pinocchio may not use an unconstrained conda solve"
    assert_replacement_mujoco_archive_is_rejected()
    assert_mujoco_version_override_cannot_escape_deps_dir()
    assert_mujoco_checksum_failure_preserves_existing_prefix()
    assert_existing_mujoco_prefix_requires_matching_provenance()
    assert_pinocchio_requires_verified_archive()
    assert_pinocchio_requires_archive_inputs()
    assert_existing_pinocchio_prefix_requires_verified_archive_inputs()
    assert_existing_pinocchio_prefix_requires_matching_provenance()
    assert_non_cmake_pinocchio_prefix_is_rejected()
    assert_interface_only_pinocchio_prefix_is_rejected()
    assert_pinocchio_probe_rejects_foreign_target_dependency()
    assert_pinocchio_probe_rejects_foreign_direct_link_dependency()
    assert_pinocchio_probe_rejects_foreign_legacy_link_interface()
    assert_pinocchio_probe_rejects_foreign_configured_legacy_link_interface()
    assert_pinocchio_probe_rejects_foreign_legacy_dependent_library()
    assert_pinocchio_archive_rejects_pax_nul_member()
    assert_pinocchio_verified_archive_is_not_swapped_before_extraction()
    assert_pinocchio_probe_accepts_prefix_target_property_include_reference()
    assert_verified_cmake_prefix_archive_installs()
    assert_cmake_prefix_in_lib64_installs()
    assert_fwd_header_only_pinocchio_prefix_installs()
    assert_pinocchio_probe_rejects_external_dependency_escape()
    assert_pinocchio_archive_rejects_escaping_header_symlink()
    assert_unusable_exported_pinocchio_target_is_rejected()
    assert_pinocchio_promotion_failure_restores_existing_prefix()
    assert_pinocchio_backup_cleanup_failure_commits_new_prefix()
    assert_readme_pinocchio_command_is_shell_safe()
    print("validation dependency installer contract: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
