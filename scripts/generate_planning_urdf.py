#!/usr/bin/env python3
"""Generate `urdf/lite.urdf` from the pinned `lite_urdf` submodule.

`lite.urdf` used to be a hand-carried snapshot exported from an old xacro
(commit 29e9472). That snapshot went stale: `right_elbow_pitch` kept the
pre-2026-06-29 convention (`axis 0 0 +1`, limit `[0, 2.356199]`) while
`lite_urdf` and the RoboTwin embodiment had moved to `axis 0 0 -1`,
limit `[-2.356199, 0]`. Feeding a RoboTwin/sim start state to MoveIt made
`right_elbow_pitch` look out of bounds and every right-arm plan aborted.

This script removes the hand-carry. It reads the canonical URDF straight out of
`third_party/lite_urdf` (git submodule) and applies exactly two documented
transforms:

1. **Joint naming** - `lite_urdf`'s generated variants suffix the 14 arm joints
   and the 2 gripper joints with `_joint`. This package (and the whole
   `lite_ros2` hardware stack: `bar_bringup_lite/config/lite_hardware.yaml`,
   `bar_lite_controllers.yaml`) uses the bare names, so the suffix is dropped
   for those 16 joints only. Camera, gripper-passive and gripper-tip joints
   keep their `_joint` suffix, matching both sides.

2. **Mesh package URI** - `package://lite_urdf/meshes/` becomes
   `package://lite_moveit2/meshes/`, since this package ships its own copy of
   the same STL files (verified byte-identical upstream).

Everything else is copied verbatim: no XML re-serialisation, so a future
upstream change produces a minimal, reviewable diff instead of a full reflow.

Usage
-----
    python3 scripts/generate_planning_urdf.py           # rewrite urdf/lite.urdf
    python3 scripts/generate_planning_urdf.py --check   # exit 1 if out of date

The `--check` mode is what CI runs to detect drift.
"""

from __future__ import annotations

import argparse
import filecmp
import re
import shutil
import subprocess
import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parent.parent
SUBMODULE = PKG_ROOT / "third_party" / "lite_urdf"
SOURCE_URDF = SUBMODULE / "urdf" / "lite_flash_arm_gripper.urdf"
SOURCE_MESHES = SUBMODULE / "meshes"
OUTPUT_URDF = PKG_ROOT / "urdf" / "lite.urdf"
OUTPUT_MESHES = PKG_ROOT / "urdf" / "meshes"

# The 14 arm joints + 2 gripper joints lose the `_joint` suffix.
# Joints NOT listed here keep their upstream name verbatim.
ARMS = ("left", "right")
ARM_JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch",
              "wrist_yaw", "wrist_roll", "wrist_pitch")
STRIP_SUFFIX = tuple(
    [f"{side}_{joint}_joint" for side in ARMS for joint in ARM_JOINTS]
    + [f"{side}_gripper_joint" for side in ARMS]
)

MESH_PREFIX_FROM = "package://lite_urdf/meshes/"
MESH_PREFIX_TO = "package://lite_moveit2/meshes/"


def _fail(msg: str) -> None:
    print(f"error: {msg}", file=sys.stderr)
    raise SystemExit(1)


def submodule_commit() -> str:
    """Short commit of the pinned submodule, for the provenance header."""
    try:
        out = subprocess.run(
            ["git", "-C", str(SUBMODULE), "rev-parse", "--short", "HEAD"],
            check=True, capture_output=True, text=True,
        )
        return out.stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "unknown"


def render() -> str:
    if not SOURCE_URDF.is_file():
        _fail(
            f"{SOURCE_URDF} not found.\n"
            "       Initialise the submodule first:\n"
            "         git submodule update --init --recursive"
        )

    text = SOURCE_URDF.read_text(encoding="utf-8")

    # 1. joint naming: drop the suffix, word-boundary anchored so that
    #    `<side>_gripper_passive_joint` / `_gripper_tip_middle_joint` are
    #    never touched by the `<side>_gripper_joint` rule.
    for name in sorted(STRIP_SUFFIX, key=len, reverse=True):
        bare = name[: -len("_joint")]
        text, n = re.subn(rf"(?<![\w-]){re.escape(name)}(?![\w-])", bare, text)
        if n == 0:
            _fail(f"expected joint {name!r} not present in {SOURCE_URDF.name}; "
                  "upstream renamed it - update STRIP_SUFFIX")

    # 2. mesh package URI
    if MESH_PREFIX_FROM not in text:
        _fail(f"expected mesh prefix {MESH_PREFIX_FROM!r} not found upstream")
    text = text.replace(MESH_PREFIX_FROM, MESH_PREFIX_TO)

    # 3. provenance header, inserted right after the XML declaration
    header = (
        "<!-- GENERATED FILE - DO NOT EDIT BY HAND.\n"
        f"     Source : lite_urdf @ {submodule_commit()}  "
        "urdf/lite_flash_arm_gripper.urdf\n"
        "     Regen  : python3 scripts/generate_planning_urdf.py\n"
        "     Transforms: joint-name suffix adaptation (see script docstring)\n"
        "                 + mesh package URI rewrite. Kinematics unchanged. -->\n"
    )
    if text.startswith("<?xml"):
        head, sep, rest = text.partition("\n")
        text = head + sep + header + rest
    else:
        text = header + text

    if not text.endswith("\n"):
        text += "\n"
    return text


def referenced_meshes(text: str) -> list[str]:
    return sorted({
        m.rsplit("/", 1)[-1]
        for m in re.findall(rf"{re.escape(MESH_PREFIX_TO)}([^\"]+)", text)
    })


def sync_meshes(names: list[str], check_only: bool) -> list[str]:
    """Copy referenced meshes from the submodule; return list of problems."""
    problems: list[str] = []
    OUTPUT_MESHES.mkdir(parents=True, exist_ok=True)
    for name in names:
        src = SOURCE_MESHES / name
        dst = OUTPUT_MESHES / name
        if not src.is_file():
            problems.append(f"mesh {name} referenced but missing upstream")
            continue
        if dst.is_file() and filecmp.cmp(src, dst, shallow=False):
            continue
        if check_only:
            problems.append(f"mesh {name} differs from upstream")
        else:
            shutil.copy2(src, dst)
    return problems


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true",
                    help="only verify that the checked-in files are up to date")
    args = ap.parse_args()

    text = render()
    meshes = referenced_meshes(text)
    problems = sync_meshes(meshes, check_only=args.check)

    if args.check:
        if not OUTPUT_URDF.is_file():
            problems.append(f"{OUTPUT_URDF.relative_to(PKG_ROOT)} is missing")
        elif OUTPUT_URDF.read_text(encoding="utf-8") != text:
            problems.append(
                f"{OUTPUT_URDF.relative_to(PKG_ROOT)} is out of date with the submodule"
            )
        if problems:
            print("lite.urdf drift detected:", file=sys.stderr)
            for p in problems:
                print(f"  - {p}", file=sys.stderr)
            print("\nRegenerate with: python3 scripts/generate_planning_urdf.py",
                  file=sys.stderr)
            return 1
        print(f"OK: urdf/lite.urdf and {len(meshes)} meshes match lite_urdf "
              f"@ {submodule_commit()}")
        return 0

    OUTPUT_URDF.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_URDF.write_text(text, encoding="utf-8")
    for p in problems:
        print(f"warning: {p}", file=sys.stderr)
    print(f"wrote {OUTPUT_URDF.relative_to(PKG_ROOT)} from lite_urdf @ "
          f"{submodule_commit()} ({len(meshes)} meshes synced)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
