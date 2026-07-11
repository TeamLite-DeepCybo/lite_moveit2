"""Move left/right arm to a named SRDF pose."""

from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node

from lite_moveit2.move_group_client import LiteArmMotion


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('arm', choices=['left', 'right', 'left_arm', 'right_arm'])
    parser.add_argument('pose', help='SRDF group_state name, e.g. home, ready, selfie')
    parser.add_argument('--velocity-scale', type=float, default=0.2)
    parser.add_argument('--acceleration-scale', type=float, default=0.2)
    args = parser.parse_args(argv)

    rclpy.init()
    node = Node('lite_move_to_pose')
    motion = LiteArmMotion(node)
    try:
        motion.wait_for_servers()
        node.get_logger().info(
            f'Moving {args.arm} to named pose {args.pose!r}'
        )
        motion.move_arm_to_pose(
            args.arm,
            args.pose,
            velocity_scale=args.velocity_scale,
            acceleration_scale=args.acceleration_scale,
        )
        node.get_logger().info('Motion completed successfully')
        return 0
    except Exception as exc:  # noqa: BLE001 - CLI entry point
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
