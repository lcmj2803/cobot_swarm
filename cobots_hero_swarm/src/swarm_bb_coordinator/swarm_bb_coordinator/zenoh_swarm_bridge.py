#!/usr/bin/env python3
"""
Zenoh Swarm Bridge for ROS 2 Humble.

Bridges ROS 2 topics and Eclipse Zenoh key expressions:
  - Subscribes to ROS 2 /{robot_id}/odom -> Publishes Zenoh swarm/robot/{robot_id}/telemetry
  - Subscribes to Zenoh swarm/robot/{robot_id}/cmd_vel -> Publishes ROS 2 /{robot_id}/cmd_vel

Supports:
  - Single robot mode (e.g., robot_id: 'swarm_bb_01' on physical Pi Zero)
  - Multi-robot simulation mode (robot_id: 'all' bridging swarm_bb_01 to swarm_bb_06)
"""

import json
import math
import sys
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

try:
    import zenoh
except ImportError:
    zenoh = None


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert quaternion orientation to 2D yaw angle in radians."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class ZenohSwarmBridge(Node):
    """Bridge node translating between ROS 2 Humble topics and Zenoh network expressions."""

    DEFAULT_ROBOT_IDS = [f'swarm_bb_{i:02d}' for i in range(1, 7)]

    def __init__(self):
        super().__init__('zenoh_swarm_bridge')

        # Declare parameters
        self.declare_parameter('robot_id', 'all')
        self.declare_parameter('zenoh_mode', 'peer')
        self.declare_parameter('zenoh_connect', '')
        self.declare_parameter('zenoh_listen', '')

        robot_id_param = self.get_parameter('robot_id').get_parameter_value().string_value
        zenoh_mode = self.get_parameter('zenoh_mode').get_parameter_value().string_value
        zenoh_connect = self.get_parameter('zenoh_connect').get_parameter_value().string_value
        zenoh_listen = self.get_parameter('zenoh_listen').get_parameter_value().string_value

        if zenoh is None:
            self.get_logger().error(
                "Failed to import 'zenoh'. Please ensure eclipse-zenoh is installed (pip install eclipse-zenoh)."
            )
            sys.exit(1)

        # Resolve robot list
        if robot_id_param.lower() in ('all', '*', 'simulation'):
            self.target_robots: List[str] = list(self.DEFAULT_ROBOT_IDS)
            self.multi_robot_mode = True
        else:
            norm_id = robot_id_param if robot_id_param.startswith('swarm_bb_') else f'swarm_bb_{int(robot_id_param):02d}'
            self.target_robots = [norm_id]
            self.multi_robot_mode = False

        self.get_logger().info(f"Initializing Zenoh Swarm Bridge for robots: {self.target_robots}")

        # Initialize Zenoh session
        zenoh_config = zenoh.Config()
        if zenoh_mode:
            try:
                zenoh_config.insert_json5('mode', json.dumps(zenoh_mode))
            except Exception:
                pass
        if zenoh_connect:
            try:
                zenoh_config.insert_json5('connect/endpoints', json.dumps([zenoh_connect]))
            except Exception:
                pass
        if zenoh_listen:
            try:
                zenoh_config.insert_json5('listen/endpoints', json.dumps([zenoh_listen]))
            except Exception:
                pass

        try:
            self.session = zenoh.open(zenoh_config)
            self.get_logger().info("Zenoh session opened successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to open Zenoh session: {e}")
            raise e

        # Setup QoS profiles
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Maps for publishers and subscriptions
        self.cmd_vel_publishers: Dict[str, rclpy.publisher.Publisher] = {}
        self.odom_subscribers = []
        self.zenoh_subscribers = []

        # Setup ROS 2 publishers and subscribers for each robot
        for rid in self.target_robots:
            # 1. ROS 2 cmd_vel publisher
            cmd_topic = f'/{rid}/cmd_vel'
            pub = self.create_publisher(Twist, cmd_topic, cmd_vel_qos)
            self.cmd_vel_publishers[rid] = pub
            self.get_logger().info(f"Registered ROS 2 cmd_vel publisher: {cmd_topic}")

            # 2. ROS 2 odom subscriber -> Zenoh telemetry publisher
            odom_topic = f'/{rid}/odom'
            sub = self.create_subscription(
                Odometry,
                odom_topic,
                lambda msg, robot_name=rid: self._on_ros_odom(msg, robot_name),
                odom_qos
            )
            self.odom_subscribers.append(sub)
            self.get_logger().info(f"Subscribed to ROS 2 odom topic: {odom_topic}")

        # Setup Zenoh subscriber for velocity commands
        if self.multi_robot_mode:
            cmd_key_expr = 'swarm/robot/*/cmd_vel'
            z_sub = self.session.declare_subscriber(cmd_key_expr, self._on_zenoh_cmd_vel_wildcard)
            self.zenoh_subscribers.append(z_sub)
            self.get_logger().info(f"Zenoh subscriber registered for: '{cmd_key_expr}'")
        else:
            single_rid = self.target_robots[0]
            cmd_key_expr = f'swarm/robot/{single_rid}/cmd_vel'
            z_sub = self.session.declare_subscriber(
                cmd_key_expr,
                lambda sample, robot_name=single_rid: self._on_zenoh_cmd_vel_single(sample, robot_name)
            )
            self.zenoh_subscribers.append(z_sub)
            self.get_logger().info(f"Zenoh subscriber registered for: '{cmd_key_expr}'")

    def _on_ros_odom(self, msg: Odometry, robot_id: str):
        """Translate ROS 2 Odometry message into a Zenoh telemetry packet."""
        try:
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            yaw = quaternion_to_yaw(qx, qy, qz, qw)

            # Extract timestamp in seconds
            stamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)

            payload_dict = {
                'id': robot_id,
                'timestamp': stamp,
                'x': float(msg.pose.pose.position.x),
                'y': float(msg.pose.pose.position.y),
                'theta': float(yaw),
                'vx': float(msg.twist.twist.linear.x),
                'vy': float(msg.twist.twist.linear.y),
                'wz': float(msg.twist.twist.angular.z)
            }
            payload_str = json.dumps(payload_dict)

            key_expr = f'swarm/robot/{robot_id}/telemetry'
            self.session.put(key_expr, payload_str)
        except Exception as e:
            self.get_logger().error(f"Error publishing odom to Zenoh for {robot_id}: {e}")

    def _on_zenoh_cmd_vel_wildcard(self, sample):
        """Handle incoming Zenoh cmd_vel matching swarm/robot/*/cmd_vel."""
        try:
            key = str(sample.key_expr)
            # Parse robot ID from key: swarm/robot/{robot_id}/cmd_vel
            parts = key.strip('/').split('/')
            if len(parts) >= 3 and parts[0] == 'swarm' and parts[1] == 'robot':
                robot_id = parts[2]
                self._dispatch_cmd_vel(sample, robot_id)
        except Exception as e:
            self.get_logger().error(f"Error handling wildcard Zenoh cmd_vel: {e}")

    def _on_zenoh_cmd_vel_single(self, sample, robot_id: str):
        """Handle incoming Zenoh cmd_vel for a specific robot."""
        self._dispatch_cmd_vel(sample, robot_id)

    def _dispatch_cmd_vel(self, sample, robot_id: str):
        """Parse Zenoh payload and publish Twist to the target robot's ROS 2 topic."""
        pub = self.cmd_vel_publishers.get(robot_id)
        if pub is None:
            return

        try:
            payload_bytes = sample.payload.to_bytes()
            data = json.loads(payload_bytes.decode('utf-8'))

            twist = Twist()
            # Support both {linear: 0.2, angular: 0.1} and {linear: {x: 0.2}, angular: {z: 0.1}}
            if isinstance(data.get('linear'), (int, float)):
                twist.linear.x = float(data.get('linear', 0.0))
            elif isinstance(data.get('linear'), dict):
                twist.linear.x = float(data['linear'].get('x', 0.0))
                twist.linear.y = float(data['linear'].get('y', 0.0))
                twist.linear.z = float(data['linear'].get('z', 0.0))

            if isinstance(data.get('angular'), (int, float)):
                twist.angular.z = float(data.get('angular', 0.0))
            elif isinstance(data.get('angular'), dict):
                twist.angular.x = float(data['angular'].get('x', 0.0))
                twist.angular.y = float(data['angular'].get('y', 0.0))
                twist.angular.z = float(data['angular'].get('z', 0.0))

            pub.publish(twist)
        except Exception as e:
            self.get_logger().warn(f"Failed to decode or publish cmd_vel for {robot_id}: {e}")

    def destroy_node(self):
        """Clean up Zenoh session on node destruction."""
        self.get_logger().info("Closing Zenoh session...")
        try:
            if hasattr(self, 'session') and self.session is not None:
                self.session.close()
        except Exception as e:
            self.get_logger().warn(f"Error closing Zenoh session: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    bridge_node = ZenohSwarmBridge()
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
