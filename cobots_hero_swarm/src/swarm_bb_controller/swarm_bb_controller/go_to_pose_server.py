import math 
import time 

import rclpy 
from rclpy.action import ActionServer
from rclpy.node import Node 

from swarm_bb_interfaces.action import GoToPose 
from swarm_bb_interfaces.msg import RobotState

from std_srvs.srv import Trigger 

class GoToPoseServer(Node):
    
    def __init__(self):
        super().__init__('go_to_pose_server')
        self.current_x = 0.0 
        self.current_y = 0.0
        
        self.declare_parameter('robot_id','swarm_bb_01')
        self.declare_parameter('battery_percentage',0.30)

        self.current_vx = 0.0
        self.current_vy = 0.0
        
        self.motion_state = RobotState.STATE_IDLE
        
        self.battery_level = float(
        self.get_parameter('battery_percentage').value
        )

        self.idle_discharge_rate = 0.002
        self.moving_discharge_rate = 0.005
        
        self.charging_rate = 0.02
        self.target_battery_level = 0.90

        self.last_battery_update = time.monotonic()
        
        
        
                
        self.speed = 0.5 
        self.step_period = 0.2
        
        self.state_publisher = self.create_publisher(
            RobotState,
            'state',
            10
        )
        
        self.state_timer = self.create_timer(
            self.step_period,
            self.publish_state
        )
                
        self.action_server = ActionServer(
            self,
            GoToPose,
            'go_to_pose',
            self.execute_callback
        )

        
        self.charging_service = self.create_service(
            Trigger,
            'start_charging',
            self.start_charging_callback
        )
        
        
        self.get_logger().info(
            'Servidor de accionón GoToPose iniciado'
        )
        
        
        
    def execute_callback(self, goal_handle):
            target_x = goal_handle.request.target_pose.pose.position.x
            target_y = goal_handle.request.target_pose.pose.position.y

            tolerance = goal_handle.request.position_tolerance

            self.get_logger().info(
                f'Objetivo recibido: ({target_x:.2f}, {target_y:.2f})'
        )

            feedback = GoToPose.Feedback()

            distance = math.hypot(
                target_x - self.current_x,
                target_y - self.current_y
        )

            while distance > tolerance:
                step = min(
                    self.speed * self.step_period,
                    distance
            )

                delta_x = step * (
                    target_x - self.current_x
                ) / distance

                delta_y = step * (
                    target_y - self.current_y
                ) / distance

                self.current_vx = delta_x / self.step_period
                self.current_vy = delta_y / self.step_period

                self.current_x += delta_x
                self.current_y += delta_y
    
                self.update_battery()
                self.motion_state = RobotState.STATE_MOVING

                distance = math.hypot(
                    target_x - self.current_x,
                    target_y - self.current_y
            )

                state_message = self.publish_state()

                feedback.current_pose.header = state_message.header
                feedback.current_pose.pose = state_message.pose
                feedback.distance_remaining = distance

                goal_handle.publish_feedback(feedback)

                time.sleep(self.step_period)

            self.update_battery()
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.motion_state = RobotState.STATE_IDLE

            self.publish_state()

            goal_handle.succeed()

            result = GoToPose.Result()
            result.success = True
            result.message = 'Objetivo alcanzado'

            self.get_logger().info(result.message)

            return result
        
    
    def publish_state(self):
        self.update_battery()

        message = RobotState()

        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'

        message.robot_id = self.get_parameter('robot_id').value

        message.pose.position.x = self.current_x
        message.pose.position.y = self.current_y
        message.pose.orientation.w = 1.0

        message.velocity.linear.x = self.current_vx
        message.velocity.linear.y = self.current_vy

        message.battery_percentage = self.battery_level
        message.state = self.motion_state

        self.state_publisher.publish(message)

        return message
    
    def start_charging_callback(self,request,response):
        
        if self.motion_state == RobotState.STATE_CHARGING:
            response.success = True
            response.message = 'El robot ya está en CHARGING'
            return response 
        
        if self.motion_state != RobotState.STATE_IDLE:
            response.success = False 
            response.message = 'El robot debe estar detenido '
            return response 
        
        self.update_battery()
        
        self.current_vx = 0.0 
        self.current_vy = 0.0 
        self.motion_state = RobotState.STATE_CHARGING
        
        
        self.publish_state()
        
        response.success = True 
        response.message = 'Estado CHARGING activado'


        self.get_logger().info(response.message) 
        
        return response 
    
        
    def update_battery(self):
        now = time.monotonic()

        elapsed = now - self.last_battery_update
        self.last_battery_update = now

        if self.motion_state == RobotState.STATE_CHARGING:
            previous_level = self.battery_level

            self.battery_level = min(
                self.target_battery_level,
                self.battery_level + self.charging_rate * elapsed
            )

            if (
                previous_level < self.target_battery_level
                and self.battery_level >= self.target_battery_level
            ):
                self.get_logger().info(
                    'Batería al 90%: objetivo de recarga alcanzado'
                )

            return

        if self.motion_state == RobotState.STATE_MOVING:
            discharge_rate = self.moving_discharge_rate
        else:
            discharge_rate = self.idle_discharge_rate

        consumed = discharge_rate * elapsed

        self.battery_level = max(
            0.0,
            self.battery_level - consumed
        )
       

        
        
        
def main(args=None):
    rclpy.init(args= args)
    node = GoToPoseServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.try_shutdown()
    
if __name__ == '__main__':
    main()
    