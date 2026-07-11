"""Load named group states from the Lite MoveIt SRDF file."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def default_srdf_path() -> Path:
    share = Path(get_package_share_directory('lite_moveit2'))
    return share / 'config' / 'lite_000_asm.srdf'


def load_named_states(srdf_path: Path | None = None) -> dict[tuple[str, str], dict[str, float]]:
    """Return {(group, pose_name): {joint_name: value}}."""
    path = srdf_path or default_srdf_path()
    root = ET.parse(path).getroot()
    states: dict[tuple[str, str], dict[str, float]] = {}
    for group_state in root.findall('group_state'):
        group = group_state.get('group')
        name = group_state.get('name')
        if group is None or name is None:
            continue
        joints = {
            joint.get('name'): float(joint.get('value'))
            for joint in group_state.findall('joint')
            if joint.get('name') is not None and joint.get('value') is not None
        }
        states[(group, name)] = joints
    return states


def get_named_state(
    group: str,
    pose_name: str,
    srdf_path: Path | None = None,
) -> dict[str, float]:
    states = load_named_states(srdf_path)
    key = (group, pose_name)
    if key not in states:
        known = sorted(name for grp, name in states if grp == group)
        raise KeyError(
            f"Unknown pose '{pose_name}' for group '{group}'. Known: {known}"
        )
    return states[key]


ARM_GROUPS = {
    'left': 'left_arm',
    'right': 'right_arm',
    'left_arm': 'left_arm',
    'right_arm': 'right_arm',
}


def normalize_arm_group(arm: str) -> str:
    key = arm.strip().lower()
    if key not in ARM_GROUPS:
        raise ValueError(f"Unknown arm '{arm}'. Use 'left' or 'right'.")
    return ARM_GROUPS[key]
