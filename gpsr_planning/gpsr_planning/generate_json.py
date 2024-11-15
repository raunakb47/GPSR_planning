import json
from xml.dom import minidom
from rclpy.node import Node
import rclpy
from ament_index_python.packages import get_package_share_directory
import time
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
        initial_time = time.time()
        rclpy.spin_until_future_complete(self, future)
        elapsed_time = time.time() - initial_time
        if future.result() is not None:
            response = future.result()
            result = self.convert_response_to_json(response)
            return {"command": command, "plan": json.loads(response.plan_json), "elapsed_time": elapsed_time}
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

        result = {"actions": actions}

        return result


def main(args=None):
    rclpy.init(args=args)
    node = GenerateJSONNode()

    input_file = get_package_share_directory("gpsr_planning") + "/test/gpsr_dataset0_eval.json"
    output_filename = "./model1_output.json"

    action_json = {"actions": []}

    with open(input_file, "r") as f:
        data = json.load(f)
        commands = [entry["command"] for entry in data]
        for idx, command in enumerate(commands):
            command = command.strip()
            print(f"Generating plan for command: {command}")
            result = node.send_request(command)
            result["gt"] = data[idx]["output"]
            print(f'Elapsed time: {result["elapsed_time"]:0.2f} s')
            print(f'Remaining time: {(len(commands) - idx) * result["elapsed_time"] / 60:0.2f} min')
            print("")
            # print(result['actions'])
            action_json["actions"].append(result)
            
            if idx == 300:
                break

    with open(output_filename, "w") as f:
        json.dump(action_json, f, indent=4)

    rclpy.shutdown()
