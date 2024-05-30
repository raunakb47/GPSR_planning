
import json
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

import ament_index_python
from xml.dom import minidom

from gpsr_msgs.srv import GeneratePlan
from gpsr_planning.gpsr_planner import GpsrPlanner


class GpsrPlanningNode(Node):

    def __init__(self) -> None:
        super().__init__("gpsr_planning_node")

        robot_actions_path = ament_index_python.get_package_share_directory(
            "gpsr_planning") + "/params/robot_actions.json"

        waypoints_path = ament_index_python.get_package_share_directory(
            "gpsr_planning") + "/params/waypoints.json"

        self.gpsr_planner = GpsrPlanner(robot_actions_path, waypoints_path)
        self._srv = self.create_service(
            GeneratePlan, "gpsr_planning", self._execute_cb,
            callback_group=ReentrantCallbackGroup())

    def _execute_cb(self, request: GeneratePlan.Request, response: GeneratePlan.Response) -> GeneratePlan.Response:

        self.get_logger().info("Generating GPSR Plan")

        plan, _ = self.gpsr_planner.send_prompt(request.command)
        self.get_logger().info(json.dumps(plan, indent=4))

        for action in plan["actions"]:
            action_name = list(action.keys())[1]
            response.action_list.append(action_name)

        response.bt_xml = self.action_parser(plan)
        self.get_logger().info(response.bt_xml)

        return response

    def action_parser(self, plan: Dict) -> str:

        bt_xml = minidom.Document()

        root_element = bt_xml.createElement("root")
        root_element.setAttribute("main_tree_to_execute", "BehaviorTree")
        bt_element = bt_xml.createElement("BehaviorTree")
        bt_element.setAttribute("ID", "BehaviorTree")

        sequence_element = bt_xml.createElement("Sequence")

        for action in plan["actions"]:

            action_name = list(action.keys())[1]
            action_args = action[action_name]

            action_element = bt_xml.createElement("SubTree")
            action_element.setAttribute(
                "ID", f"{action_name.replace('_', ' ').title().replace(' ', '')}Tree")
            action_element.setAttribute("__shared_blackboard", "true")

            for arg_key in action_args:
                arg_value = action_args[arg_key]
                action_element.setAttribute(arg_key, arg_value)

            sequence_element.appendChild(action_element)

            include = bt_xml.createElement("include")
            include.setAttribute(
                "path", f"{ament_index_python.get_package_share_directory('gpsr_planning')}/bt_xml/{action_name}.xml")
            root_element.appendChild(include)

        bt_element.appendChild(sequence_element)
        root_element.appendChild(bt_element)
        bt_xml.appendChild(root_element)
        str_bt = bt_xml.toprettyxml(indent="\t")

        return str_bt


def main():
    rclpy.init()
    node = GpsrPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
