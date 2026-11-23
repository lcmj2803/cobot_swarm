import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from swarm_bb_interfaces.action import GoToPose


class GoToPoseClient(Node):

    def __init__(self):
        super().__init__('go_to_pose_client')

        self.finished = False

        self.action_client = ActionClient(
            self,
            GoToPose,
            'go_to_pose'
        )

    def send_goal(self):
        while rclpy.ok():
            if self.action_client.wait_for_server(timeout_sec=1.0):
                break

            self.get_logger().info(
                'Esperando al servidor GoToPose...'
            )

        if not rclpy.ok():
            return

        goal = GoToPose.Goal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal.target_pose.pose.position.x = 2.0
        goal.target_pose.pose.position.y = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        goal.position_tolerance = 0.05
        goal.yaw_tolerance = 0.1

        self.get_logger().info(
            'Enviando objetivo: (2.00, 0.00)'
        )

        self.send_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        self.send_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warning('Objetivo rechazado')
            self.finished = True
            return

        self.get_logger().info('Objetivo aceptado')

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(
            self.result_callback
        )

    def feedback_callback(self, message):
        feedback = message.feedback

        self.get_logger().info(
            f'Distancia restante: '
            f'{feedback.distance_remaining:.2f} m'
        )

    def result_callback(self, future):
        result = future.result().result

        self.get_logger().info(
            f'Resultado: {result.message} | '
            f'success={result.success}'
        )

        self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseClient()

    try:
        node.send_goal()

        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.5)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()