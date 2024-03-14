import rclpy
from rclpy.node import Node

class GPSRPlanning(Node):

    def __init__(self):
        super().__init__('gpsr_planning')

        # param names
        commands_param_name = "commands"

        # declaring params
        self.declare_parameter(commands_param_name, None)

        # getting params
        self.__commands = self.get_parameter(commands_param_name).get_parameter_value().string_array_value




def main(args=None):
    rclpy.init(args=args)
    node = GPSRPlanning()
    node.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()