"""
Aggregate Webots trial logs under ``Results/`` into ``BOV.csv``.

Run from this directory (``SpikeWave``) after simulations have written logs, or pass
``--spike-dir`` pointing at this folder from the repository root.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import pandas as pd

LOG = logging.getLogger(__name__)

INDEX = [
    "Trial Number",
    "Robot Number",
    "Time taken (s)",
    "Wall Obstacles (#)",
    "Number of Collisions (#)",
    "Area occupancy (%)",
    "Efficiency (Overlap)(%)",
    "Efficiency (Overall area covered by all robots) (%)",
]


def load_free_cell_count(map_path: Path) -> int:
    """Count traversable cells (values < 241) in the tab-separated grid map."""
    df = pd.read_csv(map_path, sep="\t", header=None)
    return int((df < 241).sum().sum())


def read_trail(file_name: Path) -> list[float | int]:
    df = pd.read_csv(file_name, header=None)
    time = float(df.iloc[0, 0].split("goals: ")[1])
    wall = int(df.iloc[1, 0].split("Obstacles ")[1])
    robot = int(df.iloc[2, 0].split("Obstacles ")[1])
    return [time, wall, robot]


def read_efficiency(file_names: list[Path]) -> tuple[int, int]:
    df = None
    for idx, file_name in enumerate(file_names):
        chunk = pd.read_csv(file_name, header=None, sep=" ")
        chunk.drop([64], axis=1, inplace=True)
        chunk[chunk != 0] = 1
        if idx == 0:
            df = chunk
        else:
            df = df + chunk
    assert df is not None
    overall = int((df >= 1).sum().sum())
    overlap = int((df >= 2).sum().sum())
    return overall, overlap


def read_occupancy(file_name: Path, free_cell_count: int) -> float:
    df = pd.read_csv(file_name, header=None, sep=" ")
    df.drop([64], axis=1, inplace=True)
    occupancy = int((df > 0).sum().sum())
    if free_cell_count <= 0:
        raise ValueError("free_cell_count must be positive (check map file).")
    return occupancy / free_cell_count


def build_column_names(strategy_dirs: list[str]) -> list[str]:
    columns: list[str] = []
    for name in strategy_dirs:
        columns.extend([f"{name}_1Robot_trial{i + 1}_robot1" for i in range(5)])
        columns.extend(
            [
                f"{name}_3Robot_trial{i + 1}_robot{j + 1}"
                for i in range(5)
                for j in range(3)
            ]
        )
        columns.extend(
            [
                f"{name}_5Robot_trial{i + 1}_robot{j + 1}"
                for i in range(5)
                for j in range(5)
            ]
        )
    return columns


def aggregate(
    spike_dir: Path,
    map_path: Path,
    results_dir: Path,
    output_csv: Path,
) -> None:
    free_cells = load_free_cell_count(map_path)
    strategy_dirs = sorted(
        p.name for p in results_dir.iterdir() if p.is_dir()
    )
    if not strategy_dirs:
        raise FileNotFoundError(
            f"No strategy subfolders found under {results_dir}. "
            "Run Webots trials first or restore the Results/ directory."
        )

    columns = build_column_names(strategy_dirs)
    df = pd.DataFrame(0, columns=columns, index=INDEX)

    for strategy in strategy_dirs:
        for n_robots in (1, 3, 5):
            for trial_num in range(1, 6):
                robots: list[str] = []
                path_robots: list[Path] = []
                for robot in range(1, n_robots + 1):
                    rel = (
                        results_dir
                        / strategy
                        / f"Result {n_robots}"
                        / str(trial_num)
                    )
                    trail = rel / f"outputLogFor{robot}with{n_robots}Robots.txt"
                    occ = rel / f"outputLogFor{robot}with{n_robots}RobotsAreaOccupancy.txt"
                    info = read_trail(trail)
                    col = f"{strategy}_{n_robots}Robot_trial{trial_num}_robot{robot}"
                    df.loc["Trial Number", col] = trial_num
                    df.loc["Robot Number", col] = robot
                    df.loc["Time taken (s)", col] = info[0]
                    df.loc["Wall Obstacles (#)", col] = info[1]
                    df.loc["Number of Collisions (#)", col] = info[2]
                    df.loc["Area occupancy (%)", col] = read_occupancy(occ, free_cells)
                    path_robots.append(occ)
                    robots.append(col)
                overall, overlap = read_efficiency(path_robots)
                df.loc["Efficiency (Overlap)(%)", robots] = overlap
                df.loc["Efficiency (Overall area covered by all robots) (%)", robots] = overall

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(output_csv)
    LOG.info("Wrote %s", output_csv)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--spike-dir",
        type=Path,
        default=here,
        help="Directory containing this script, map files, and Results/ (default: script dir).",
    )
    parser.add_argument(
        "--map-file",
        type=str,
        default="edited map2.txt",
        help="Map file name relative to --spike-dir.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output CSV path (default: <spike-dir>/BOV.csv).",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Print log messages to stderr.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.WARNING,
        format="%(levelname)s: %(message)s",
    )
    spike_dir = args.spike_dir.resolve()
    map_path = (spike_dir / args.map_file).resolve()
    results_dir = (spike_dir / "Results").resolve()
    output_csv = (
        args.output.resolve()
        if args.output is not None
        else (spike_dir / "BOV.csv").resolve()
    )
    if not map_path.is_file():
        LOG.error("Map file not found: %s", map_path)
        return 1
    if not results_dir.is_dir():
        LOG.error("Results directory not found: %s", results_dir)
        return 1
    try:
        aggregate(spike_dir, map_path, results_dir, output_csv)
    except (OSError, ValueError, FileNotFoundError) as exc:
        LOG.error("%s", exc)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
