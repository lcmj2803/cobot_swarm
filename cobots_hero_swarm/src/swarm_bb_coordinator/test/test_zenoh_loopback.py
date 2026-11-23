#!/usr/bin/env python3
"""
Integration test for the Zenoh bridge and coordinator communication pipeline.
Verifies:
  ROS 2 /swarm_bb_01/odom
       |
       v (zenoh_swarm_bridge)
  Zenoh swarm/robot/swarm_bb_01/telemetry
       |
       v (leader_follower_coordinator)
  Zenoh swarm/robot/swarm_bb_02/cmd_vel
       |
       v (zenoh_swarm_bridge)
  ROS 2 /swarm_bb_02/cmd_vel
"""

import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from swarm_bb_coordinator.zenoh_swarm_bridge import ZenohSwarmBridge
from swarm_bb_coordinator.leader_follower_coordinator import LeaderFollowerCoordinator


class LoopbackTester(Node):
    def __init__(self):
        super().__init__('loopback_tester')
        self.odom_pub_01 = self.create_publisher(Odometry, '/swarm_bb_01/odom', 10)
        self.odom_pub_02 = self.create_publisher(Odometry, '/swarm_bb_02/odom', 10)
        self.received_cmd_vel = []
        self.cmd_sub_02 = self.create_subscription(
            Twist, '/swarm_bb_02/cmd_vel', self._cmd_cb, 10
        )

    def _cmd_cb(self, msg: Twist):
        self.received_cmd_vel.append(msg)


def main():
    rclpy.init()

    bridge = ZenohSwarmBridge()
    coordinator = LeaderFollowerCoordinator()
    tester = LoopbackTester()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(coordinator)
    executor.add_node(tester)

    print("Sending simulated odometry for Leader (0,0) and Follower 2 (-0.8, 0.8)...")

    # Leader at (0, 0, yaw=0)
    odom1 = Odometry()
    odom1.header.stamp = tester.get_clock().now().to_msg()
    odom1.pose.pose.position.x = 0.0
    odom1.pose.pose.position.y = 0.0
    odom1.pose.pose.orientation.w = 1.0

    # Follower 2 at (-0.8, 0.8) - Target in wedge is (-0.35, 0.35), so it should move toward it!
    odom2 = Odometry()
    odom2.header.stamp = tester.get_clock().now().to_msg()
    odom2.pose.pose.position.x = -0.8
    odom2.pose.pose.position.y = 0.8
    odom2.pose.pose.orientation.w = 1.0

    start_time = time.time()
    success = False

    while time.time() - start_time < 3.0:
        tester.odom_pub_01.publish(odom1)
        tester.odom_pub_02.publish(odom2)
        executor.spin_once(timeout_sec=0.05)
        if len(tester.received_cmd_vel) > 0:
            success = True
            break

    print(f"Test completed in {time.time() - start_time:.2f}s. Received commands: {len(tester.received_cmd_vel)}")
    if success:
        cmd = tester.received_cmd_vel[0]
        print(f"SUCCESS: Follower 2 received /swarm_bb_02/cmd_vel -> linear.x={cmd.linear.x:.3f}, angular.z={cmd.angular.z:.3f}")

    bridge.destroy_node()
    coordinator.destroy_node()
    tester.destroy_node()
    rclpy.shutdown()

    assert success, "Failed to receive /swarm_bb_02/cmd_vel through the Zenoh loop!"


if __name__ == '__main__':
    main()
