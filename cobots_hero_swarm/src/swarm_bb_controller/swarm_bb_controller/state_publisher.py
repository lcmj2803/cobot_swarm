import rclpy
from rclpy.node import Node

from swarm_bb_interfaces.msg import RobotState

class StatePublisher(Node):
    
    def __init__(self):
        super().__init__('state_publisher')
        
        self.publisher = self.create_publisher(
            RobotState,
            'state',
            10
        )
        
        self.timer = self.create_timer(
            1.0,
            self.publish_state
        )
        
        self.get_logger().info('State Publisher Iniciado')
        
    def publish_state(self):
        message =RobotState()
        
        #Momento y sistema de coordenadas
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id ='map'
        
        #Identificación 
        message.robot_id = 'swarm_bb_01'
        
        #Pose provisional
        message.pose.position.x = 0.0 
        message.pose.position.y = 0.0
        message.pose.position.z = 0.0 
        
        #Quat sin rotacion
        message.pose.orientation.x = 0.0
        message.pose.orientation.y = 0.0
        message.pose.orientation.z = 0.0 
        message.pose.orientation.w = 1.0
        
        #velocidad provisional 
        message.velocity.linear.x = 0.0 
        message.velocity.angular.z = 0.0
        
        #Estado Provisional 
        message.battery_percentage = 1.0
        message.state = RobotState.STATE_IDLE
        
        self.publisher.publish(message)
        
        self.get_logger().info(
            f'Estado publicado: {message.robot_id}'
        )
        
def main(args=None):
    rclpy.init(args=args)
        
    node = StatePublisher()
        
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroynode()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()