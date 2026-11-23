import rclpy
from rclpy.node import Node 

from swarm_bb_interfaces.msg import RobotState

class StateMonitor (Node):
    def __init__(self):
        super().__init__('state_monitor')
        
        self.subscription = self.create_subscription(
            RobotState,
            'state',
            self.state_callback,
            10
        )
    
    def state_callback(self,message):
        state_names= {
            RobotState.STATE_UNKNOWN: 'UNKNOWN',
            RobotState.STATE_IDLE: 'IDLE',
            RobotState.STATE_MOVING: 'MOVING',
            RobotState.STATE_CHARGING: 'CHARGING',
            RobotState.STATE_ERROR:'ERROR',
        }
        
        state_name = state_names.get(
            message.state,
            'INVALID'
        )
        
        self.get_logger().info(
            f'Robot:{message.robot_id} |'
            f'Bateria:{message.battery_percentage * 100:.1f}% |'
            f'Posicion:({message.pose.position.x:.2f}) | '
            f'{message.pose.position.y:.2f} | '
            f'Estado:{state_name}'            
        )
        
def main(args=None):
    rclpy.init(args=args)
        
    node = StateMonitor()
        
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
    
    
            
            
            
