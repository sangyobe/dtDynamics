#!/usr/bin/env python3
"""Lock the ART Bug Finder and selected MISRA C++:2023 checker profile."""

from __future__ import annotations

import xml.etree.ElementTree as element_tree
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
CHECKERS = ROOT / "polyspace" / "checkers.xml"
EXPECTED_MISRA_CPP_2023 = {
    "0.0.1", "0.0.2", "0.1.1", "0.1.2", "0.2.1", "0.2.2", "0.2.3", "0.2.4",
    "4.1.1", "4.1.2", "4.1.3", "4.6.1",
    "5.0.1", "5.7.1", "5.7.3", "5.10.1", "5.13.1", "5.13.2", "5.13.3", "5.13.4", "5.13.5", "5.13.6", "5.13.7",
    "6.0.1", "6.0.2", "6.0.3", "6.0.4", "6.2.1", "6.2.2", "6.2.3", "6.2.4",
    "6.4.1", "6.4.2", "6.4.3", "6.5.1", "6.5.2", "6.7.1", "6.7.2",
    "6.8.1", "6.8.2", "6.8.3", "6.8.4", "6.9.1", "6.9.2",
    "7.0.1", "7.0.2", "7.0.3", "7.0.4", "7.0.5", "7.0.6", "7.11.1", "7.11.2", "7.11.3",
    "8.0.1", "8.1.1", "8.1.2",
    "8.2.1", "8.2.2", "8.2.3", "8.2.4", "8.2.5", "8.2.6", "8.2.7", "8.2.8", "8.2.9", "8.2.10", "8.2.11",
    "8.3.1", "8.3.2", "8.7.1", "8.7.2", "8.9.1", "8.14.1", "8.18.1", "8.18.2", "8.19.1", "8.20.1",
    "9.2.1", "9.3.1", "9.4.1", "9.4.2", "9.5.1", "9.5.2", "9.6.1", "9.6.2", "9.6.3", "9.6.4", "9.6.5",
    "10.0.1", "10.1.1", "10.1.2", "10.2.1", "10.2.2", "10.2.3", "10.3.1", "10.4.1",
    "11.3.1", "11.3.2", "11.6.1", "11.6.2", "11.6.3",
    "12.2.1", "12.2.2", "12.2.3", "12.3.1",
    "13.1.1", "13.1.2", "13.3.1", "13.3.2", "13.3.3", "13.3.4",
    "14.1.1",
    "15.0.1", "15.0.2", "15.1.1", "15.1.2", "15.1.3", "15.1.4", "15.1.5",
    "16.5.1", "16.5.2", "16.6.1",
    "17.8.1",
    "18.1.1", "18.1.2", "18.3.1", "18.3.2", "18.3.3", "18.4.1", "18.5.1", "18.5.2",
    "19.0.1", "19.0.2", "19.0.3", "19.0.4",
    "19.1.1", "19.1.2", "19.1.3",
    "19.2.1", "19.2.2", "19.2.3",
    "19.3.1", "19.3.2", "19.3.3", "19.3.4", "19.3.5",
    "19.6.1",
    "21.2.1", "21.2.2", "21.2.3", "21.2.4",
    "21.6.1", "21.6.2", "21.6.3", "21.6.4", "21.6.5",
    "21.10.1", "21.10.2", "21.10.3",
    "22.3.1", "22.4.1",
    "23.11.1",
    "24.5.1", "24.5.2",
    "25.5.1", "25.5.2", "25.5.3",
    "26.3.1",
    "28.3.1", "28.6.1", "28.6.2", "28.6.3", "28.6.4",
    "30.0.1", "30.0.2",
    "D0.3.1", "D0.3.2", "D5.7.2", "D15.8.1",
}


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
