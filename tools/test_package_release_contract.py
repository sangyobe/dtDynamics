#!/usr/bin/env python3
"""Regression tests for exact tag identity in source release archives."""

from __future__ import annotations

import shutil
import subprocess
import sys
import tarfile
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
RELEASE_SCRIPT = ROOT / "tools" / "package_release.sh"


def run(command: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=cwd,
        check=False,
        capture_output=True,
        text=True,
    )


def create_repository() -> tuple[tempfile.TemporaryDirectory[str], Path]:
    temporary = tempfile.TemporaryDirectory()
    repository = Path(temporary.name) / "repository"
    (repository / "tools").mkdir(parents=True)
    shutil.copy2(RELEASE_SCRIPT, repository / "tools" / "package_release.sh")
    (repository / "tools" / "package_release.sh").chmod(0o755)
    (repository / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.16)\n"
        "project(dtDynamics VERSION 0.1.3 LANGUAGES NONE)\n",
        encoding="utf-8",
    )
    (repository / "payload.txt").write_text("release payload\n", encoding="utf-8")
    for command in (
        ["git", "init", "-q"],
        ["git", "config", "user.name", "release-contract-test"],
        ["git", "config", "user.email", "release-contract-test@example.invalid"],
        ["git", "add", "."],
        ["git", "commit", "-qm", "initial release candidate"],
    ):
        result = run(command, repository)
        assert result.returncode == 0, result.stdout + result.stderr
    return temporary, repository


def package(repository: Path, version: str = "0.1.3") -> subprocess.CompletedProcess[str]:
    return run(
        [
            "bash",
            "tools/package_release.sh",
            "--skip-audit",
            "--version",
            version,
            "--output-dir",
            str(repository.parent / "dist"),
        ],
        repository,
    )


def assert_untagged_head_is_rejected() -> None:
    temporary, repository = create_repository()
    try:
        result = package(repository)
        assert result.returncode != 0, result.stdout + result.stderr
        assert "exact release tag" in (result.stdout + result.stderr).lower()
    finally:
        temporary.cleanup()


def assert_tag_on_other_commit_is_rejected() -> None:
    temporary, repository = create_repository()
    try:
        result = run(["git", "tag", "v0.1.3"], repository)
        assert result.returncode == 0, result.stdout + result.stderr
        (repository / "payload.txt").write_text("changed after tag\n", encoding="utf-8")
        result = run(["git", "add", "payload.txt"], repository)
        assert result.returncode == 0, result.stdout + result.stderr
        result = run(["git", "commit", "-qm", "post-tag change"], repository)
        assert result.returncode == 0, result.stdout + result.stderr

        result = package(repository)
        assert result.returncode != 0, result.stdout + result.stderr
        assert "does not resolve to HEAD" in (result.stdout + result.stderr)
    finally:
        temporary.cleanup()


def assert_version_must_match_project_version() -> None:
    temporary, repository = create_repository()
    try:
        result = run(["git", "tag", "v0.1.3"], repository)
        assert result.returncode == 0, result.stdout + result.stderr
        result = package(repository, version="0.1.4")
        assert result.returncode != 0, result.stdout + result.stderr
        assert "does not match project version" in (result.stdout + result.stderr)
    finally:
        temporary.cleanup()


def assert_exact_tagged_head_archives_the_tagged_tree() -> None:
    temporary, repository = create_repository()
    try:
        result = run(["git", "tag", "v0.1.3"], repository)
        assert result.returncode == 0, result.stdout + result.stderr
        expected_head = run(["git", "rev-parse", "HEAD"], repository)
        assert expected_head.returncode == 0, expected_head.stdout + expected_head.stderr

        result = package(repository)
        assert result.returncode == 0, result.stdout + result.stderr
        archive = repository.parent / "dist" / "dtDynamics-0.1.3.tar.gz"
        checksum = archive.parent / f"{archive.name}.sha256"
        assert archive.is_file()
        assert checksum.is_file()
        with tarfile.open(archive, "r:gz") as tar:
            names = tar.getnames()
        assert "dtDynamics-0.1.3/payload.txt" in names
        assert "dtDynamics-0.1.3/.git" not in names
        archive_commit = run(
            [
                "bash",
                "-c",
                'gzip -dc -- "$1" | git get-tar-commit-id',
                "bash",
                str(archive),
            ],
            repository,
        )
        assert archive_commit.returncode == 0, archive_commit.stdout + archive_commit.stderr
        assert archive_commit.stdout.strip() == expected_head.stdout.strip()
    finally:
        temporary.cleanup()


def main() -> None:
    assert_untagged_head_is_rejected()
    assert_tag_on_other_commit_is_rejected()
    assert_version_must_match_project_version()
    assert_exact_tagged_head_archives_the_tagged_tree()
    print("package release contract tests: PASS")


if __name__ == "__main__":
    main()
