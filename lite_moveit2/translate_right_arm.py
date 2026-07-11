"""Translate the right arm EE in Cartesian space."""

from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node

from lite_moveit2.move_group_client import LiteArmMotion


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dx', type=float, default=0.0, help='Translation X [m]')
    parser.add_argument('--dy', type=float, default=0.0, help='Translation Y [m]')
    parser.add_argument('--dz', type=float, default=0.0, help='Translation Z [m]')
    parser.add_argument(
        '--frame',
        choices=['ee', 'world'],
        default='ee',
        help='Interpret dx/dy/dz in EE-local or world frame',
    )
    parser.add_argument('--eef-step', type=float, default=0.005)
    parser.add_argument('--min-fraction', type=float, default=0.95)
    parser.add_argument(
        '--avoid-collisions',
        action='store_true',
        help='Enable collision checking during Cartesian interpolation (default: off)',
    )
    args = parser.parse_args(argv)

    if args.dx == 0.0 and args.dy == 0.0 and args.dz == 0.0:
        parser.error('At least one of --dx/--dy/--dz must be non-zero')

    rclpy.init()
    node = Node('lite_translate_right_arm')
    motion = LiteArmMotion(node)
    try:
        motion.wait_for_servers()
        node.get_logger().info(
            f'Translating right arm by ({args.dx}, {args.dy}, {args.dz}) m in {args.frame} frame'
        )
        fraction = motion.translate_right_arm(
            args.dx,
            args.dy,
            args.dz,
            frame=args.frame,
            eef_step=args.eef_step,
            min_fraction=args.min_fraction,
            avoid_collisions=args.avoid_collisions,
        )
        node.get_logger().info(f'Cartesian motion completed (fraction={fraction:.1%})')
        return 0
    except Exception as exc:  # noqa: BLE001 - CLI entry point
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
