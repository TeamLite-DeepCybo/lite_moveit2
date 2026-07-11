"""MoveIt move_group client helpers for Lite dual-arm motions."""

from __future__ import annotations

import time
from typing import Iterable

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath
from tf2_ros import Buffer, TransformListener

from .srdf_states import get_named_state, normalize_arm_group

SUCCESS = MoveItErrorCodes.SUCCESS

RIGHT_EE_LINK = 'right_gripper_tip_middle_link'
PLANNING_FRAME = 'world'


class MoveGroupClient:
    """Thin wrapper around move_group plan / execute interfaces."""

    def __init__(self, node: Node) -> None:
        self._node = node
        self._logger = node.get_logger()
        self._move_client = ActionClient(node, MoveGroup, '/move_action')
        self._execute_client = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
        self._cartesian_client = node.create_client(
            GetCartesianPath, '/compute_cartesian_path'
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

    def wait_for_servers(self, timeout_sec: float = 30.0) -> None:
        if not self._move_client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError('move_group action /move_action not available')
        if not self._execute_client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError('execute action /execute_trajectory not available')
        if not self._cartesian_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError('service /compute_cartesian_path not available')

    def move_to_named_pose(
        self,
        arm: str,
        pose_name: str,
        *,
        velocity_scale: float = 0.2,
        acceleration_scale: float = 0.2,
    ) -> None:
        group = normalize_arm_group(arm)
        joints = get_named_state(group, pose_name)
        request = self._build_joint_goal_request(
            group,
            joints,
            velocity_scale=velocity_scale,
            acceleration_scale=acceleration_scale,
        )
        self._run_move_group(request)

    def translate_right_arm(
        self,
        dx: float,
        dy: float,
        dz: float,
        *,
        frame: str = 'ee',
        eef_step: float = 0.01,
        jump_threshold: float = 0.0,
        min_fraction: float = 0.95,
    ) -> float:
        """Translate the right EE by (dx, dy, dz) meters. Returns path fraction."""
        start_pose = self._lookup_pose(PLANNING_FRAME, RIGHT_EE_LINK)
        target_pose = self._offset_pose(start_pose, dx, dy, dz, frame=frame)

        trajectory, fraction = self._plan_cartesian_path(
            group='right_arm',
            link=RIGHT_EE_LINK,
            waypoints=[start_pose, target_pose],
            eef_step=eef_step,
            jump_threshold=jump_threshold,
        )
        if fraction < min_fraction:
            raise RuntimeError(
                f'Cartesian path achieved only {fraction:.1%} '
                f'(required {min_fraction:.1%})'
            )
        self._execute_trajectory(trajectory)
        return fraction

    def _build_joint_goal_request(
        self,
        group: str,
        joints: dict[str, float],
        *,
        velocity_scale: float,
        acceleration_scale: float,
    ) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        req = goal.request
        req.group_name = group
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = velocity_scale
        req.max_acceleration_scaling_factor = acceleration_scale
        req.start_state = RobotState()
        req.start_state.is_diff = True

        constraints = Constraints()
        for joint_name, position in joints.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = position
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)
        return goal

    def _run_move_group(self, goal: MoveGroup.Goal) -> None:
        send_future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('MoveGroup goal rejected')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        result = result_future.result().result
        if result.error_code.val != SUCCESS:
            raise RuntimeError(
                f'MoveGroup failed: code={result.error_code.val} '
                f'message={result.error_code.message!r}'
            )

    def _plan_cartesian_path(
        self,
        *,
        group: str,
        link: str,
        waypoints: Iterable[Pose],
        eef_step: float,
        jump_threshold: float,
    ):
        request = GetCartesianPath.Request()
        request.header.frame_id = PLANNING_FRAME
        request.group_name = group
        request.link_name = link
        request.waypoints = list(waypoints)
        request.max_step = float(eef_step)
        request.jump_threshold = float(jump_threshold)
        request.avoid_collisions = True
        request.start_state = RobotState()
        request.start_state.is_diff = True

        future = self._cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future)
        response = future.result()
        if response is None:
            raise RuntimeError('GetCartesianPath call failed')
        if response.error_code.val != SUCCESS:
            raise RuntimeError(
                f'GetCartesianPath failed: code={response.error_code.val} '
                f'message={response.error_code.message!r}'
            )
        return response.solution, response.fraction

    def _execute_trajectory(self, trajectory) -> None:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self._execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('ExecuteTrajectory goal rejected')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        result = result_future.result().result
        if result.error_code.val != SUCCESS:
            raise RuntimeError(
                f'ExecuteTrajectory failed: code={result.error_code.val} '
                f'message={result.error_code.message!r}'
            )

    def _lookup_pose(self, target_frame: str, source_frame: str) -> Pose:
        self._wait_for_transform(target_frame, source_frame)
        transform = self._tf_buffer.lookup_transform(
            target_frame, source_frame, rclpy.time.Time()
        )
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def _wait_for_transform(
        self,
        target_frame: str,
        source_frame: str,
        timeout_sec: float = 10.0,
    ) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self._tf_buffer.can_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.0),
            ):
                return
            rclpy.spin_once(self._node, timeout_sec=0.1)
        raise RuntimeError(
            f'TF unavailable: {target_frame} <- {source_frame}. '
            'Is move_group / robot_state_publisher running?'
        )

    def _offset_pose(
        self,
        start_pose: Pose,
        dx: float,
        dy: float,
        dz: float,
        *,
        frame: str,
    ) -> Pose:
        frame = frame.strip().lower()
        if frame in ('ee', 'eef', 'tool', 'right_ee'):
            return self._offset_in_ee_frame(start_pose, dx, dy, dz)
        if frame in ('world', 'base', PLANNING_FRAME):
            target = Pose()
            target.orientation = start_pose.orientation
            target.position.x = start_pose.position.x + dx
            target.position.y = start_pose.position.y + dy
            target.position.z = start_pose.position.z + dz
            return target
        raise ValueError("frame must be 'ee' or 'world'")

    def _offset_in_ee_frame(self, start_pose: Pose, dx: float, dy: float, dz: float) -> Pose:
        # Rotate the local translation by the EE orientation, keep orientation fixed.
        q = start_pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        rot = [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
        local = [dx, dy, dz]
        world_delta = [
            rot[0][0] * local[0] + rot[0][1] * local[1] + rot[0][2] * local[2],
            rot[1][0] * local[0] + rot[1][1] * local[1] + rot[1][2] * local[2],
            rot[2][0] * local[0] + rot[2][1] * local[1] + rot[2][2] * local[2],
        ]
        target = Pose()
        target.orientation = start_pose.orientation
        target.position.x = start_pose.position.x + world_delta[0]
        target.position.y = start_pose.position.y + world_delta[1]
        target.position.z = start_pose.position.z + world_delta[2]
        return target


class LiteArmMotion:
    """High-level API intended for Agent function-calling wrappers."""

    def __init__(self, node: Node) -> None:
        self._client = MoveGroupClient(node)

    def wait_for_servers(self, timeout_sec: float = 30.0) -> None:
        self._client.wait_for_servers(timeout_sec=timeout_sec)

    def move_arm_to_pose(
        self,
        arm: str,
        pose_name: str,
        *,
        velocity_scale: float = 0.2,
        acceleration_scale: float = 0.2,
    ) -> None:
        """Move left/right arm to a named SRDF pose."""
        self._client.move_to_named_pose(
            arm,
            pose_name,
            velocity_scale=velocity_scale,
            acceleration_scale=acceleration_scale,
        )

    def translate_right_arm(
        self,
        dx: float,
        dy: float,
        dz: float,
        *,
        frame: str = 'ee',
        eef_step: float = 0.01,
        min_fraction: float = 0.95,
    ) -> float:
        """Translate right arm EE by meters in EE-local or world frame."""
        return self._client.translate_right_arm(
            dx,
            dy,
            dz,
            frame=frame,
            eef_step=eef_step,
            min_fraction=min_fraction,
        )
