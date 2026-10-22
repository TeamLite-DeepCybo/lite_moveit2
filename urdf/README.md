# Lite MoveIt Planning URDF

Planning model for MoveIt 2, **generated from the `lite_urdf` submodule** rather than
carried by hand.

| Path | Purpose |
|------|---------|
| `lite.urdf` | **Generated.** Flattened planning URDF (`world_root` root, no `ros2_control`) |
| `meshes/` | Generated copy of the 23 STL assets referenced by `lite.urdf` |
| `../third_party/lite_urdf` | Git submodule — the canonical source |

Do not edit `lite.urdf` or `meshes/` by hand. Regenerate them:

```bash
git submodule update --init --recursive
python3 scripts/generate_planning_urdf.py          # write
python3 scripts/generate_planning_urdf.py --check  # verify (CI)
```

## Source

| | |
|---|---|
| Repo | `https://github.com/TeamLite-DeepCybo/lite_urdf` |
| Path in submodule | `third_party/lite_urdf/urdf/lite_flash_arm_gripper.urdf` |
| Pinned commit | recorded by the submodule pointer (`git submodule status`) |
| Robot name | `lite_000_asm` |

## Transforms applied by the generator

The generator is a **textual** transform — no XML re-serialisation — so an upstream
change produces a minimal, reviewable diff.

1. **Joint naming.** `lite_urdf`'s generated variants suffix the 14 arm joints and the
   2 gripper joints with `_joint`. This package keeps the bare names, because the whole
   `lite_ros2` hardware stack uses them
   (`bar_bringup_lite/config/lite_hardware.yaml`, `bar_lite_controllers.yaml`).
   Only those 16 joints are renamed; `head_camera_joint`,
   `*_gripper_passive_joint`, `*_gripper_tip_middle_joint` and
   `*_wrist_camera_joint` keep their upstream names.
2. **Mesh package URI.** `package://lite_urdf/meshes/` → `package://lite_moveit2/meshes/`.
   The mesh files themselves are byte-identical to upstream; the copy exists because
   meshes must resolve through the ament index of *this* package.

Nothing else is altered: origins, axes, limits, link geometry and dynamics are exactly
what `lite_urdf` ships.

## Why this file is generated (regression history)

`lite.urdf` used to be a snapshot exported from `lite_urdf`'s `urdf/lite.urdf.xacro`
at commit `29e9472` (*"fix: use SI gripper opening limits"*). That export went stale:

- The `right_elbow_pitch` convention was flipped in the **generated URDF variants** on
  2026-06-29 (`5183ad7`, `axis 0 0 -1`, limit `[-2.356199, 0]`), but the xacro that the
  snapshot was taken from still said `axis 0 0 1`, limit `[0, 2.356199]`.
- `lite_urdf` later removed that xacro, leaving the generated variants as the only
  source of truth — and MoveIt frozen on the abandoned convention.

Consequences:

- Handing MoveIt a robot/sim start state with `right_elbow_pitch < 0` made the joint look
  out of bounds. MoveIt rejects an out-of-bounds *start state* and aborts the plan, so
  **every right-arm plan failed** while the left arm worked.
- The two `right_arm` named poses in `config/lite_000_asm.srdf` were authored in the old
  convention (`ready` = `+1.7`, `selfie` = `+2.0177`) and became invalid. They are now
  `-1.7` and `-2.0177` — the same physical posture expressed in the current convention.

Regenerating from the submodule removes the whole class of failure: the planning model
can only ever match the canonical URDF that the rest of the stack consumes.

> **Note on `motorDirection`.** The `<hardware>` blocks are copied through verbatim
> because URDF consumers ignore them and keeping them makes the diff honest. They carry
> no kinematic meaning — `1bdca07` (*"Reverse right arm motor directions"*) only flips
> `<hardware><motorDirection>` for the right arm and changes no axis, limit or origin.
> That commit is a **driver-level** change owned by `lite_ros2`, not by this package.

## Mesh URI Convention

Meshes use `package://lite_moveit2/meshes/<file>.stl`.

After `colcon build`, meshes resolve via the ament index — **do not** point
`ROS_PACKAGE_PATH` at `urdf/`; that path has no `package.xml` and will not work for
`package://lite_moveit2/...`.

## Structure

- Root link: `world_root`; the SRDF adds a fixed virtual joint `world` → `world_root`.
- 24 links / 23 joints: 14 arm + 2 gripper + 2 mimic + 5 fixed (cameras, gripper tips).
- No `ros2_control` block — execution stays in `lite_ros2`; this package only *plans*.
