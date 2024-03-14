import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from gpsr_msgs.action import ExecutePlan

class GPSRPlanning(Node):

    def __init__(self):
        super().__init__('planning_node')
        self._action_server = ActionServer(self, ExecutePlan, 'gpsr_planning', self._execute_cb)
        
    def _execute_cb(self, goal_handle) -> ExecutePlan.Result:
        self.get_logger().info(f'Executing plan with the following command: {goal_handle.request.command}')
        
        return ExecutePlan.Result()

def main(args=None):
    rclpy.init(args=args)
    node = GPSRPlanning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()