import rclpy 
from rclpy.node import Node 
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from action_msgs.msg import GoalStatus

from swarm_bb_interfaces.srv import RequestChargingStation
from swarm_bb_interfaces.action import GoToPose
from swarm_bb_interfaces.msg import RobotState

from std_srvs.srv import Trigger 

class ChargingMissionClient(Node):
    
    def __init__(self):
        super().__init__('charging_mission_client')

        self.declare_parameter('robot_id', 'swarm_bb_01')
        self.declare_parameter('low_battery_threshold', 0.20)

        self.finished = False
        self.succeeded = False
        self.station_id = ''

        self.latest_state = None
        self.mission_started = False

        self.charging_client = self.create_client(
        RequestChargingStation,
        '/swarm/request_charging_station'
    )

        self.action_client = ActionClient(
        self,
        GoToPose,
        'go_to_pose'
    )

        self.state_subscription = self.create_subscription(
        RobotState,
        'state',
        self.state_callback,
        10
    )

        self.start_charging_client = self.create_client(
        Trigger,
        'start_charging'
    )


        self.mission_timer = self.create_timer(
        1.0,
        self.start_mission
    )

        self.get_logger().info(
        'Esperando el estado del robot...'
    )
        
    def state_callback(self, message):
        robot_id = self.get_parameter('robot_id').value

        if message.robot_id != robot_id:
            return

        self.latest_state = message
        
        self.get_logger().info(
            f'Estado recibido: {message.robot_id} | '
            f'Batería: {message.battery_percentage * 100:.1f}% | '
            f'Estado: {message.state}'
        )
        
    def start_mission(self):
        if self.mission_started or self.finished:
            return

        if self.latest_state is None:
            return

        state = self.latest_state

        threshold = self.get_parameter(
            'low_battery_threshold'
        ).value

        if state.battery_percentage > threshold:
            return

        if state.state != RobotState.STATE_IDLE:
            return

        if not self.charging_client.service_is_ready():
            self.get_logger().info(
                'Esperando al coordinador de carga...'
            )
            return

        if not self.action_client.server_is_ready():
            self.get_logger().info(
                'Esperando al servidor GoToPose...'
            )
            return

        if not self.start_charging_client.service_is_ready():
            self.get_logger().info(
                'Esperando al servicio de inicio de carga...'
            )
            return
        
        
        
        request = RequestChargingStation.Request()
        request.robot_id = state.robot_id
        request.battery_percentage = state.battery_percentage

        self.mission_started = True

        self.get_logger().info(
            f'Batería: {request.battery_percentage * 100:.1f}% | '
            f'Solicitando carga para {request.robot_id}'
        )

        self.station_future = self.charging_client.call_async(
            request
        )

        self.station_future.add_done_callback(
            self.station_response_callback
        )
            
    def station_response_callback(self, future):
            response = future.result()

            if not response.accepted:
                self.fail(
                    f'Solicitud rechazada: {response.message}'
            )
                return

            self.station_id = response.station_id

            goal = GoToPose.Goal()

            goal.target_pose = response.station_pose
            goal.position_tolerance = 0.05
            goal.yaw_tolerance = 0.1

            self.get_logger().info(
                f'Estación asignada: {self.station_id} | '
                f'Destino: '
                f'({goal.target_pose.pose.position.x:.2f}, '
                f'{goal.target_pose.pose.position.y:.2f})'
        )

            self.goal_future = self.action_client.send_goal_async(
                goal,
                feedback_callback=self.feedback_callback
        )

            self.goal_future.add_done_callback(
                self.goal_response_callback
        )  
            
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.fail('El servidor GoToPose rechazó el objetivo')
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
        response = future.result()

        if (
            response.status == GoalStatus.STATUS_SUCCEEDED
            and response.result.success
        ):
            self.get_logger().info(
                f'Llegada a {self.station_id}. '
                'Solicitando inicio de carga...'
            )

            if not self.start_charging_client.service_is_ready():
                self.fail(
                    'El servicio start_charging no está disponible'
                )
                return

            self.start_charging_future = (
                self.start_charging_client.call_async(
                    Trigger.Request()
                )
            )

            self.start_charging_future.add_done_callback(
                self.start_charging_response_callback
            )
        else:
            self.fail(
                f'Movimiento no completado: '
                f'{response.result.message}'
            )
            
    def start_charging_response_callback(self,future):
        try:
            response = future.result()
        except Exception as error:
            self.fail(f'Error al iniciar la carga: {error}')
            return

        if not response.success:
            self.fail(
                f'No se pudo iniciar la carga: {response.message}'
            )
            return

        self.get_logger().info(
            f'{response.message} en {self.station_id}'
        )

        self.succeeded = True
        self.finished = True
        
        
    def fail(self, message):
        self.get_logger().error(message)
        self.finished = True
            
def main(args=None):
        rclpy.init(args=args)

        node = ChargingMissionClient()

        try:
        
            while rclpy.ok() and not node.finished:
                rclpy.spin_once(node, timeout_sec=0.5)


        except (KeyboardInterrupt, ExternalShutdownException):
            pass


        except Exception as error:
            node.fail(f'Error durante la misión: {error}')

        finally:
            node.action_client.destroy()
            node.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()

        return 0 if node.succeeded else 1


if __name__ == '__main__':
    raise SystemExit(main())

