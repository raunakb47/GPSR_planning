import json
from xml.dom import minidom
from rclpy.node import Node
import rclpy
from ament_index_python.packages import get_package_share_directory

from gpsr_msgs.srv import GeneratePlan

class GenerateJSONNode(Node):
    def __init__(self) -> None:
        super().__init__("generate_json_node")
        self._srv_client = self.create_client(GeneratePlan, "gpsr_planning")
        while not self._srv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self._srv_req = GeneratePlan.Request()

    def send_request(self, command: str) -> any:
        self._srv_req.command = command
        future = self._srv_client.call_async(self._srv_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            result = self.convert_response_to_json(response)
            return {
                "command": command,
                "result": result
            }
        else:
            self.get_logger().error("service call failed %r" % (future.exception(),))

    def convert_response_to_json(self, response: GeneratePlan.Response) -> dict:
        xml = minidom.parseString(response.bt_xml)
        sequence = xml.getElementsByTagName("Sequence")[0]
        children = sequence.childNodes
        actions = []
        args = []
        for child in children:
            if child.nodeType == child.ELEMENT_NODE:
                if child.tagName == "SetBlackboard":
                    args.append(child.getAttribute("value"))
                elif child.tagName == "SubTree":
                    action = child.getAttribute("ID")
                    for arg in args:
                        action += f" {arg.replace(' ', '_')}"
                    actions.append(action)
                    args = []
        
        result = {
            "actions": actions
        }

        return result
    
def main(args=None):
    rclpy.init(args=args)
    node = GenerateJSONNode()

    input_file = get_package_share_directory("gpsr_planning") + "/test/short.txt"
    output_filename = './commands.json'

    action_json = {"actions": []}

    with open(input_file, 'r') as f:
        commands = f.readlines()
        for command in commands:
            command = command.strip()
            print(f"Generating plan for command: {command}")
            result = node.send_request(command)
            # print(result['actions'])
            action_json["actions"].append(result)
    
    with open(output_filename, 'w') as f:
        json.dump(action_json, f, indent=4)
    
    rclpy.shutdown()