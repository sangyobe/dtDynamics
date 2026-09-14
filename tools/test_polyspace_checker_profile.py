#!/usr/bin/env python3
"""Lock the ART Bug Finder and selected MISRA C++:2023 checker profile."""

from __future__ import annotations

import xml.etree.ElementTree as element_tree
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
CHECKERS = ROOT / "polyspace" / "checkers.xml"
EXPECTED_MISRA_CPP_2023 = {"6.8.2", "8.18.1", "11.6.2", "25.5.2", "25.5.3"}


def standard(root: element_tree.Element, name: str) -> element_tree.Element:
    result = root.find(f"./standard[@name='{name}']")
    assert result is not None, f"missing standard: {name}"
    return result


def main() -> None:
    root = element_tree.parse(CHECKERS).getroot()

    bug_finder = standard(root, "Bug Finder Findings")
    enabled_bug_finder = [
        check.attrib["id"]
        for check in bug_finder.findall(".//check")
        if check.attrib.get("state") == "on"
    ]
    assert enabled_bug_finder, "Bug Finder Findings must retain at least one enabled checker"

    misra_cpp_2023 = standard(root, "MISRA C++:2023")
    enabled_misra = {
        check.attrib["id"]
        for check in misra_cpp_2023.findall(".//check")
        if check.attrib.get("state") == "on"
    }
    assert enabled_misra == EXPECTED_MISRA_CPP_2023, (
        f"unexpected MISRA C++:2023 subset: {sorted(enabled_misra)}"
    )

    misra_cpp_2008 = standard(root, "MISRA C++:2008")
    assert misra_cpp_2008.attrib.get("state") == "off", "MISRA C++:2008 must remain disabled"

    print(
        "Polyspace checker profile: "
        f"bug_finder_enabled={len(enabled_bug_finder)} "
        f"misra_cpp_2023={','.join(sorted(enabled_misra))}"
    )


if __name__ == "__main__":
    main()
