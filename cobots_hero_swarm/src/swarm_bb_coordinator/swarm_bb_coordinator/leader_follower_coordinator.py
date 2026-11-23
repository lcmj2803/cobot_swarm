#!/usr/bin/env python3
"""
Leader-Follower Swarm Coordinator with Artificial Potential Field (APF) Collision Evasion.

Subscribes to Zenoh telemetry: swarm/robot/*/telemetry
Computes formation geometry and APF repulsion vectors.
Publishes velocity commands to Zenoh: swarm/robot/{id}/cmd_vel
"""

import json
import math
import sys
import threading
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

try:
    import zenoh
except ImportError:
    zenoh = None


class RobotState:
    """Stores the latest state of a single robot."""

    def __init__(self, robot_id: str):
        self.robot_id = robot_id
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_update_time = 0.0
        self.is_valid = False

    def update(self, x: float, y: float, theta: float, vx: float, vy: float, wz: float, timestamp: float):
        self.x = x
        self.y = y
        self.theta = theta
        self.vx = vx
        self.vy = vy
        self.wz = wz
        self.last_update_time = timestamp
        self.is_valid = True


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_formation_target(
    leader_x: float, leader_y: float, leader_theta: float, dx_rel: float, dy_rel: float
) -> Tuple[float, float]:
    """
    Rotate relative body frame offset by leader heading and add to leader position.
    Target = Leader + R(theta) * Offset
    """
    cos_t = math.cos(leader_theta)
    sin_t = math.sin(leader_theta)
    target_x = leader_x + (cos_t * dx_rel - sin_t * dy_rel)
    target_y = leader_y + (sin_t * dx_rel + cos_t * dy_rel)
    return target_x, target_y


def compute_apf_repulsion(
    pos_i: Tuple[float, float],
    other_positions: List[Tuple[float, float]],
    d_safe: float = 0.28,
    k_rep: float = 0.08,
    max_rep_mag: float = 0.5
) -> Tuple[float, float]:
    """
    Calculate repulsive velocity vector from neighboring robots within d_safe.
    F_rep = sum( k_rep * (1/d - 1/d_safe) * (r_ij / d^3) )
    """
    vx_rep = 0.0
    vy_rep = 0.0
    xi, yi = pos_i

    for xj, yj in other_positions:
        rx = xi - xj
        ry = yi - yj
        dist = math.hypot(rx, ry)

        # Active avoidance zone
        if 1e-4 < dist < d_safe:
            # Repulsive magnitude
            rep_factor = k_rep * (1.0 / dist - 1.0 / d_safe) * (1.0 / (dist * dist))
            vx_rep += rep_factor * (rx / dist)
            vy_rep += rep_factor * (ry / dist)

    # Clamp total repulsive force magnitude
    rep_mag = math.hypot(vx_rep, vy_rep)
    if rep_mag > max_rep_mag:
        scale = max_rep_mag / rep_mag
        vx_rep *= scale
        vy_rep *= scale

    return vx_rep, vy_rep


def unicycle_controller(
    x_curr: float,
    y_curr: float,
    theta_curr: float,
    vx_des: float,
    vy_des: float,
    leader_theta: float,
    dist_to_target: float,
    k_theta: float = 2.0,
    v_max: float = 0.22,
    w_max: float = 1.5,
    pos_tolerance: float = 0.03
) -> Tuple[float, float]:
    """
    Map desired Cartesian velocity vector (v_des_x, v_des_y) into unicycle commands (v, omega).
    """
    if dist_to_target < pos_tolerance:
        # At target: stop forward motion and align heading with leader
        e_theta = normalize_angle(leader_theta - theta_curr)
        w_cmd = max(min(k_theta * e_theta, w_max), -w_max)
        return 0.0, w_cmd

    des_speed = math.hypot(vx_des, vy_des)
    target_heading = math.atan2(vy_des, vx_des)
    e_theta = normalize_angle(target_heading - theta_curr)

    # Angular speed proportional to heading error
    w_cmd = max(min(k_theta * e_theta, w_max), -w_max)

    # Modulate forward speed: slow down when heading error is large
    # cos(e_theta) becomes negative if facing backwards -> do not drive backward
    heading_factor = max(0.0, math.cos(e_theta))
    v_cmd = min(des_speed * heading_factor, v_max)

    return v_cmd, w_cmd


class LeaderFollowerCoordinator(Node):
    """Coordinates a 6-robot swarm using Zenoh messaging."""

    # Predefined relative offsets: {follower_id: (dx_rel, dy_rel)}
    WEDGE_OFFSETS = {
        'swarm_bb_02': (-0.35, +0.35),   # Left wing 1
        'swarm_bb_03': (-0.35, -0.35),   # Right wing 1
        'swarm_bb_04': (-0.70, +0.70),   # Left wing 2
        'swarm_bb_05': (-0.70, -0.70),   # Right wing 2
        'swarm_bb_06': (-0.70, 0.00),    # Center rear
    }

    COLUMN_OFFSETS = {
        'swarm_bb_02': (-0.35, 0.00),
        'swarm_bb_03': (-0.70, 0.00),
        'swarm_bb_04': (-1.05, 0.00),
        'swarm_bb_05': (-1.40, 0.00),
        'swarm_bb_06': (-1.75, 0.00),
    }

    LINE_OFFSETS = {
        'swarm_bb_02': (0.00, +0.35),
        'swarm_bb_03': (0.00, -0.35),
        'swarm_bb_04': (0.00, +0.70),
        'swarm_bb_05': (0.00, -0.70),
        'swarm_bb_06': (0.00, +1.05),
    }

    def __init__(self):
        super().__init__('leader_follower_coordinator')

        # Declare parameters
        self.declare_parameter('leader_id', 'swarm_bb_01')
        self.declare_parameter('formation_type', 'wedge')
        self.declare_parameter('k_p', 1.2)
        self.declare_parameter('k_theta', 2.2)
        self.declare_parameter('d_safe', 0.28)
        self.declare_parameter('k_rep', 0.08)
        self.declare_parameter('v_max', 0.22)
        self.declare_parameter('w_max', 1.5)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('zenoh_mode', 'peer')
        self.declare_parameter('zenoh_connect', '')

        self.leader_id = self.get_parameter('leader_id').get_parameter_value().string_value
        self.formation_type = self.get_parameter('formation_type').get_parameter_value().string_value.lower()
        self.k_p = self.get_parameter('k_p').get_parameter_value().double_value
        self.k_theta = self.get_parameter('k_theta').get_parameter_value().double_value
        self.d_safe = self.get_parameter('d_safe').get_parameter_value().double_value
        self.k_rep = self.get_parameter('k_rep').get_parameter_value().double_value
        self.v_max = self.get_parameter('v_max').get_parameter_value().double_value
        self.w_max = self.get_parameter('w_max').get_parameter_value().double_value
        rate_hz = self.get_parameter('control_rate_hz').get_parameter_value().double_value
        zenoh_mode = self.get_parameter('zenoh_mode').get_parameter_value().string_value
        zenoh_connect = self.get_parameter('zenoh_connect').get_parameter_value().string_value

        self.get_logger().info(
            f"Starting Swarm Coordinator: Leader={self.leader_id}, Formation={self.formation_type}, Rate={rate_hz}Hz"
        )

        # State storage and thread safety
        self.lock = threading.Lock()
        self.robot_states: Dict[str, RobotState] = {}
        for i in range(1, 7):
            rid = f'swarm_bb_{i:02d}'
            self.robot_states[rid] = RobotState(rid)

        # Initialize Zenoh
        if zenoh is None:
            self.get_logger().error("Zenoh module not available.")
            sys.exit(1)

        z_config = zenoh.Config()
        if zenoh_mode:
            try:
                z_config.insert_json5('mode', json.dumps(zenoh_mode))
            except Exception:
                pass
        if zenoh_connect:
            try:
                z_config.insert_json5('connect/endpoints', json.dumps([zenoh_connect]))
            except Exception:
                pass

        try:
            self.session = zenoh.open(z_config)
            self.get_logger().info("Connected to Zenoh network.")
        except Exception as e:
            self.get_logger().error(f"Failed to open Zenoh session: {e}")
            raise e

        # Subscribe to Zenoh telemetry for all robots
        self.telemetry_sub = self.session.declare_subscriber(
            'swarm/robot/*/telemetry', self._on_telemetry_sample
        )

        # Control loop timer
        timer_period = 1.0 / max(rate_hz, 1.0)
        self.control_timer = self.create_timer(timer_period, self._control_loop)
        self.get_logger().info(f"Control loop active at {rate_hz} Hz.")

    def _on_telemetry_sample(self, sample):
        """Parse incoming Zenoh telemetry and update robot state."""
        try:
            payload_str = sample.payload.to_bytes().decode('utf-8')
            data = json.loads(payload_str)
            rid = data.get('id')
            if rid and rid in self.robot_states:
                with self.lock:
                    self.robot_states[rid].update(
                        x=float(data.get('x', 0.0)),
                        y=float(data.get('y', 0.0)),
                        theta=float(data.get('theta', 0.0)),
                        vx=float(data.get('vx', 0.0)),
                        vy=float(data.get('vy', 0.0)),
                        wz=float(data.get('wz', 0.0)),
                        timestamp=float(data.get('timestamp', 0.0))
                    )
        except Exception as e:
            self.get_logger().warn(f"Failed to process telemetry sample: {e}")

    def _get_formation_offsets(self) -> Dict[str, Tuple[float, float]]:
        if self.formation_type == 'column':
            return self.COLUMN_OFFSETS
        elif self.formation_type == 'line':
            return self.LINE_OFFSETS
        else:
            return self.WEDGE_OFFSETS

    def _control_loop(self):
        """Execute one step of swarm formation and APF control."""
        with self.lock:
            leader = self.robot_states.get(self.leader_id)
            if leader is None or not leader.is_valid:
                return

            leader_x = leader.x
            leader_y = leader.y
            leader_theta = leader.theta

            # Collect active robot positions for APF collision check
            all_positions = {
                rid: (state.x, state.y)
                for rid, state in self.robot_states.items()
                if state.is_valid
            }

            formation_offsets = self._get_formation_offsets()

            # Iterate over followers
            for follower_id, offset in formation_offsets.items():
                follower = self.robot_states.get(follower_id)
                if follower is None or not follower.is_valid:
                    continue

                dx_rel, dy_rel = offset
                target_x, target_y = compute_formation_target(
                    leader_x, leader_y, leader_theta, dx_rel, dy_rel
                )

                # Attractive error vector
                ex = target_x - follower.x
                ey = target_y - follower.y
                dist_to_target = math.hypot(ex, ey)

                vx_att = self.k_p * ex
                vy_att = self.k_p * ey

                # APF Repulsion from all other robots (leader + other followers)
                other_pos = [pos for rid, pos in all_positions.items() if rid != follower_id]
                vx_rep, vy_rep = compute_apf_repulsion(
                    (follower.x, follower.y),
                    other_pos,
                    d_safe=self.d_safe,
                    k_rep=self.k_rep
                )

                # Total desired velocity vector
                vx_total = vx_att + vx_rep
                vy_total = vy_att + vy_rep

                # Unicycle control mapping
                v_cmd, w_cmd = unicycle_controller(
                    x_curr=follower.x,
                    y_curr=follower.y,
                    theta_curr=follower.theta,
                    vx_des=vx_total,
                    vy_des=vy_total,
                    leader_theta=leader_theta,
                    dist_to_target=dist_to_target,
                    k_theta=self.k_theta,
                    v_max=self.v_max,
                    w_max=self.w_max
                )

                # Send command over Zenoh
                cmd_payload = json.dumps({
                    'linear': round(float(v_cmd), 4),
                    'angular': round(float(w_cmd), 4)
                })
                key_expr = f'swarm/robot/{follower_id}/cmd_vel'
                try:
                    self.session.put(key_expr, cmd_payload)
                except Exception as e:
                    self.get_logger().warn(f"Failed to put cmd_vel for {follower_id}: {e}")

    def destroy_node(self):
        """Clean up resources on node destruction."""
        self.get_logger().info("Shutting down swarm coordinator...")
        try:
            if hasattr(self, 'session') and self.session is not None:
                self.session.close()
        except Exception as e:
            self.get_logger().warn(f"Error closing Zenoh session: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LeaderFollowerCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
