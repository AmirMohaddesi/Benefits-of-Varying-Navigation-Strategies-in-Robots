"""
Utility to scan a ``Results`` tree for ``AssignedGoals*.txt`` files and report paths.

Example (from ``SpikeWave`` after restoring ``Results/``):

    python ReadResults.py --root Results
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parent / "Results",
        help="Directory to search recursively (default: ./Results next to this script).",
    )
    args = parser.parse_args(argv)
    root: Path = args.root
    if not root.is_dir():
        print(f"Directory not found: {root}", file=sys.stderr)
        return 1
    matches = sorted(root.rglob("AssignedGoals*.txt"))
    if not matches:
        print(f"No AssignedGoals*.txt files under {root}")
        return 0
    for path in matches:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
