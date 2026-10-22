#!/usr/bin/env python3
"""Check that every SRDF named pose lies inside the URDF joint limits.

This is the guard for the failure that motivated the URDF regeneration: the
`right_arm` named states `ready` and `selfie` were authored under the old
`right_elbow_pitch` convention (`+1.7`, `+2.0177`) and silently became
impossible once the joint moved to `axis 0 0 -1`, limit `[-2.356199, 0]`.

MoveIt does not validate named states against URDF limits at load time, so a
pose like this only fails later, as an obscure "start state out of bounds"
planning error. Catch it in CI instead.

Also checks `config/initial_positions.yaml`.

Usage
-----
    python3 scripts/check_srdf_limits.py
    python3 scripts/check_srdf_limits.py --urdf urdf/lite.urdf --srdf config/lite_000_asm.srdf
"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parent.parent
EPS = 1e-9


def joint_limits(path: Path) -> dict[str, tuple[float, float]]:
    limits: dict[str, tuple[float, float]] = {}
    for joint in ET.parse(path).getroot().findall("joint"):
        limit = joint.find("limit")
        if limit is None:
            continue
        lo, hi = limit.get("lower"), limit.get("upper")
        if lo is None or hi is None:
            continue  # continuous joint
        limits[joint.get("name")] = (float(lo), float(hi))
    return limits


def check_srdf(srdf: Path, limits: dict[str, tuple[float, float]]) -> list[str]:
    problems: list[str] = []
    for state in ET.parse(srdf).getroot().iter("group_state"):
        group, name = state.get("group"), state.get("name")
        for joint in state.findall("joint"):
            jname = joint.get("name")
            if jname not in limits:
                continue  # virtual/unknown joint: not our business here
            value = float(joint.get("value"))
            lo, hi = limits[jname]
            if not (lo - EPS <= value <= hi + EPS):
                problems.append(
                    f"{srdf.name}: group_state {group}/{name} sets {jname}={value:g}, "
                    f"outside URDF limits [{lo:g}, {hi:g}]"
                )
    return problems


def check_initial_positions(path: Path,
                           limits: dict[str, tuple[float, float]]) -> list[str]:
    if not path.is_file():
        return []
    problems: list[str] = []
    # Deliberately avoid a PyYAML dependency: the file is a flat mapping.
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or line.endswith(":") or ":" not in line:
            continue
        name, _, raw = line.partition(":")
        name, raw = name.strip(), raw.strip()
        if name not in limits:
            continue
        try:
            value = float(raw)
        except ValueError:
            continue
        lo, hi = limits[name]
        if not (lo - EPS <= value <= hi + EPS):
            problems.append(
                f"{path.name}: {name}={value:g} outside URDF limits [{lo:g}, {hi:g}]"
            )
    return problems


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", type=Path, default=PKG_ROOT / "urdf" / "lite.urdf")
    ap.add_argument("--srdf", type=Path,
                    default=PKG_ROOT / "config" / "lite_000_asm.srdf")
    ap.add_argument("--initial-positions", type=Path,
                    default=PKG_ROOT / "config" / "initial_positions.yaml")
    args = ap.parse_args()

    for required in (args.urdf, args.srdf):
        if not required.is_file():
            print(f"error: {required} not found", file=sys.stderr)
            return 1

    limits = joint_limits(args.urdf)
    problems = check_srdf(args.srdf, limits)
    problems += check_initial_positions(args.initial_positions, limits)

    if problems:
        print("SRDF / initial-positions validation FAILED:", file=sys.stderr)
        for p in problems:
            print(f"  - {p}", file=sys.stderr)
        return 1

    print(f"OK: {args.srdf.name} named poses and initial positions are within "
          f"{args.urdf.name} joint limits ({len(limits)} bounded joints checked)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
