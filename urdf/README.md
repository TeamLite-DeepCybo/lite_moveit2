# Lite MoveIt Planning URDF Snapshot

Self-contained planning model for MoveIt 2.

## Contents

| Path | Purpose |
|------|---------|
| `lite.urdf` | Flattened planning URDF (`emit_ros2_control:=false`) |
| `meshes/` | STL assets referenced by `lite.urdf` (22 referenced assets) |
| `source/` | Historical upstream xacro snapshot |

## Source

The active snapshot uses the `lite_description20261117` link and joint
definitions imported into `lite_description/urdf/lite.urdf.xacro`. It includes
the updated wrist, finger, camera and base meshes and all three camera mounts.

Regenerate from the active workspace:

```bash
xacro src/lite_motor_controller/lite_description/urdf/lite.urdf.xacro \
  emit_ros2_control:=false \
  use_fake_hardware:=true \
  use_sim:=false \
  mode:=arms_grippers \
  | sed 's|package://lite_description/meshes/|package://lite_moveit2/meshes/|g' \
  > .lite-d1/lite_moveit2/urdf/lite.urdf
```

Copy the meshes referenced by `lite.urdf` from `lite_description/meshes/`
into this package's `urdf/meshes/`. The fixed head-camera/base pair shares
its mount geometry and is marked adjacent in the SRDF.

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
