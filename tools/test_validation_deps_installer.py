#!/usr/bin/env python3
"""Contract checks for fail-closed Pinocchio validation-dependency setup."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    installer = (ROOT / "tools/install_validation_deps.sh").read_text(encoding="utf-8")

    required = (
        'micromamba_version="2.9.0-0"',
        'micromamba_url="https://github.com/mamba-org/micromamba-releases/releases/download/${micromamba_version}/micromamba-linux-64.tar.bz2"',
        'micromamba_sha256="8761c382127e6363bd9e0a2451aa3ef90d071a79133f736e2f759a3bf13040dd"',
        'printf \'%s  %s\\n\' "${expected}" "${file}" | sha256sum --check --status',
        'download_file "${micromamba_url}" "${tmp}/micromamba.tar.bz2" || return 1',
        'verify_sha256 "${micromamba_sha256}" "${tmp}/micromamba.tar.bz2" || return 1',
        'tar -xjf "${tmp}/micromamba.tar.bz2" -C "${micromamba_root}" || return 1',
        'if [[ ! -x "${micromamba_bin}" ]]; then',
        'conda_tool="$(ensure_conda_tool)" || {',
    )
    for token in required:
        assert token in installer, f"validation dependency installer is not fail-closed: {token}"
    assert "/latest" not in installer, "validation dependency installer must not execute a mutable latest release"

    print("validation dependency installer contract: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
