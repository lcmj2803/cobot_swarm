import rclpy
from rclpy.node import Node

from swarm_bb_interfaces.srv import RequestChargingStation

class ChargingStationClient(Node):
    
    def __init__(self):
        super().__init__('charging_station_client')
        
        self.declare_parameter('robot_id','swarm_bb_01')
        self.declare_parameter('battery_percentage',0.15)
        
        self.client = self.create_client(
            RequestChargingStation,
            '/swarm/request_charging_station'
        )
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Esperando al coordinador de carga'
            )
            
        request = RequestChargingStation.Request()
        request.robot_id = self.get_parameter('robot_id').value
        
        request.battery_percentage = self.get_parameter('battery_percentage').value
        
        
        self.get_logger().info(
            f'Solicitando carga para {request.robot_id}'
        )
        
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)
        
    def response_callback(self,future):
        try:
            response = future.result()
                
            if response.accepted: 
                self.get_logger().info(
                    f'Estacion asignada:{response.station_id}'
                    f'Posicion:'
                    f'({response.station_pose.pose.position.x:.2f},'
                    f'{response.station_pose.pose.position.y:.2f})'
                    )
                            
            else: 
                self.get_logger().warning(
                    f'Solicitud Recahzada:{response.message}'
                    )
                    
        except Exception as error:
            self.get_logger().error(
                f'Error en la solicitud:{error}'
                )


def main(args=None):
    rclpy.init(args=args)
    node = ChargingStationClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()