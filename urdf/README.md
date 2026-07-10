# Lite MoveIt Planning URDF Snapshot

Self-contained planning model for MoveIt 2, isolated from `lite_ros2/lite_urdf`.

## Contents

| Path | Purpose |
|------|---------|
| `lite.urdf` | Flattened planning URDF (`emit_ros2_control:=false`) |
| `meshes/` | STL assets referenced by `lite.urdf` (23 files) |
| `source/` | Upstream xacro snapshot for traceability only |

## Source

Exported from `lite_ros2/lite_urdf` submodule:

- Commit: `29e947263f8d226553fff13dc7f6633ee9d7f5bd`
- Branch: `fix/gripper-si-position-units`
- Robot name: `lite_000_asm`
- Mode: `arms_grippers`

Regenerate:

```bash
source /opt/ros/jazzy/setup.bash
xacro /path/to/lite_ros2/lite_urdf/urdf/lite.urdf.xacro \
  emit_ros2_control:=false \
  use_fake_hardware:=true \
  use_sim:=false \
  mode:=arms_grippers \
  | sed 's|package://lite_urdf/meshes/|package://lite_moveit2/meshes/|g' \
  > lite.urdf
```

Then recopy meshes listed in `lite.urdf` into `meshes/`.

## Mesh URI Convention

Meshes use `package://lite_moveit2/meshes/<file>.stl`.

Workspace layout:

```text
where_is_my_key/          # colcon workspace root
  src/
    lite_ros2/            # control stack (includes lite_urdf submodule)
    lite_moveit2/         # this package
    Lite_Insta_Agilex_Slam/
```

Build and source (from workspace root):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to bar_bringup_lite lite_moveit2
source install/setup.bash
```

After `colcon build`, meshes resolve via the ament index — **do not** point
`ROS_PACKAGE_PATH` at `urdf/`; that path has no `package.xml` and will not
work for `package://lite_moveit2/...`.

## Notes

- No `ros2_control` block — execution stays in `lite_ros2`.
- Kinematic joints: 14 arm + 2 gripper + 2 mimic + 5 fixed (cameras/tips).
- Root link: `world_root`; add SRDF virtual joint `world` → `world_root` (fixed).
