"""Tests for PR2Maze/controllers/SpikeWave/Make_CSV.py parsing helpers."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pandas as pd
import pytest

SPIKE_DIR = Path(__file__).resolve().parents[1] / "PR2Maze" / "controllers" / "SpikeWave"
_SPEC = importlib.util.spec_from_file_location("make_csv", SPIKE_DIR / "Make_CSV.py")
assert _SPEC and _SPEC.loader
_make = importlib.util.module_from_spec(_SPEC)
sys.modules["make_csv"] = _make
_SPEC.loader.exec_module(_make)


def test_read_trail_parses_metrics(tmp_path: Path) -> None:
    log = tmp_path / "outputLogFor1with1Robots.txt"
    log.write_text(
        "time to visit all goals: 12.5\n"
        "Number of Wall Obstacles 2\n"
        "Number of Robot Obstacles 1\n",
        encoding="utf-8",
    )
    assert _make.read_trail(log) == [12.5, 2, 1]


def test_read_occupancy_normalized(tmp_path: Path) -> None:
    grid = pd.DataFrame(0, index=range(2), columns=range(65))
    grid_path = tmp_path / "occ.txt"
    grid.to_csv(grid_path, sep=" ", header=False, index=False)
    val = _make.read_occupancy(grid_path, free_cell_count=100)
    assert val == 0.0


def test_read_efficiency_overlap(tmp_path: Path) -> None:
    a = pd.DataFrame(0, index=range(2), columns=range(65))
    a.iloc[0, 0] = 5
    b = pd.DataFrame(0, index=range(2), columns=range(65))
    b.iloc[0, 0] = 3
    pa = tmp_path / "a.txt"
    pb = tmp_path / "b.txt"
    a.to_csv(pa, sep=" ", header=False, index=False)
    b.to_csv(pb, sep=" ", header=False, index=False)
    overall, overlap = _make.read_efficiency([pa, pb])
    assert overall >= overlap


def test_build_column_names_counts() -> None:
    cols = _make.build_column_names(["RT results", "SW results"])
    assert len(cols) == 2 * (5 + 15 + 25)


def test_shipped_bov_csv_has_expected_metric_rows() -> None:
    """Guardrail: committed aggregate matches the paper-facing analysis schema."""
    bov = SPIKE_DIR / "BOV.csv"
    assert bov.is_file(), "Expected PR2Maze/controllers/SpikeWave/BOV.csv in the repository"
    df = pd.read_csv(bov, index_col=0, nrows=20)
    assert "Time taken (s)" in df.index
    assert "Area occupancy (%)" in df.index
    for prefix in (
        "SW results",
        "RT results",
        "0.4RT 0.6SW results",
        "0.6RT 0.4SW results",
    ):
        assert any(str(c).startswith(prefix) for c in df.columns), f"missing columns for {prefix}"
