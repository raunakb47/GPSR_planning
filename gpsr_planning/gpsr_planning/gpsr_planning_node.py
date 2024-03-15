import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from gpsr_msgs.srv import ExecutePlan
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, Executor
from llama_msgs.action import GenerateResponse

class GPSRPlanning(Node):

    def __init__(self):
        super().__init__('planning_node')

        count_bt_node = '''count: Robot action to count the objects in the room, args json schema: {\n'object_type': {\n'title': 'Type of the object to be counted',\n'type': 'string'\n}, \n}'''

        describeO_bt_node = '''describe_object: Robot action to describe an object, args json schema: {\n'object': {\n'title': 'Object to be described'\n'type': 'string'\n}, \n}'''

        describeP_node = '''describe_person: Robot action to describe a person, args json schema: {\n'object': {\n'title': 'Person to be described'\n'type': 'string'\n}, \n}'''

        findO_bt_node = '''find_object: Robot action to find an object in the room, args json schema: {\n'object': {\n'title': 'Object to be found'\n'type': 'string'\n}, \n}'''

        findP_bt_node = '''find_person: Robot action to find a person in the room, args json schema: {\n'person': {\n'title': 'Person to be found'\n'type': 'string'\n}, \n}'''

        followP_bt_node = '''follow_person: Robot action to follow a person, args json schema: {\n'person': {\n'title': 'Object to be followed'\n'type': 'string'\n}, \n}''' 

        listen_bt_node = '''listen: Robot action to listen and transcribe what a person tells to the robot, args json schema: {\n \n}''' 

        lookAround_bt_node = '''look_around: Robot action to turn its head and look around for something, args json schema: {\n \n}'''      
        
        lookTo_bt_node = '''look_to: Robot action to make the robot look at the person, args json schema: {\n'person': {\n'title': 'Person to be looked'\n'type': 'string'\n}, \n}'''

        moveTo_bt_node = '''move_to: Robot action to move from the waypoint where it is to another waypoint, args json schema: {\n'target_waypoint': {\n'title': 'Target Waypoint'\n'type': 'string'\n}, \n}'''

        offer_bt_node = '''offer: Robot action to offer an object to a person, args json schema: {\n'object': {\n'title': 'Object to be offered'\n'type': 'string'\n}, \n}'''

        pickO_bt_node = '''pick_object: Robot action to pick and object in the location, args json schema: {\n'object': {\n'title': 'Object to be picked'\n'type': 'string'\n}, \n}'''

        placeO_bt_node = '''place_object: Robot action to place and object in the location, args json schema: {\n'object': {\n'title': 'Object to be placed'\n'type': 'string'\n}, \n}'''

        query_bt_node = '''query_object: Robot action to obtain information about what a person has told it, args json schema: {\n'text': {\n'title': 'Text to be processed'\n'type': 'string'\n}, \n}'''

        recognizeP_bt_node = '''recognize_person: Robot action to recognize a person after having met her, args json schema: {\n'person': {\n'title': 'Person to be recognized'\n'type': 'string'\n}, \n}'''

        speak_bt_node = '''speak: Robot action to speak something to a person, args json schema: {\n'text': {\n'title': 'Text to tell the person'\n'type': 'string'\n}, \n}'''

        bt_node_list = [count_bt_node, describeO_bt_node, describeP_node, followP_bt_node, findO_bt_node, findP_bt_node, listen_bt_node, lookAround_bt_node, lookTo_bt_node, moveTo_bt_node, offer_bt_node, pickO_bt_node, placeO_bt_node, query_bt_node, recognizeP_bt_node, speak_bt_node]
        self._bt_nodes = ', '.join(bt_node_list)
        self._prompt = "You are an AI planner for a robotic assistant named TIAGo. You have to generate plans, which are sequences of actions to achieve goals. Plans may contain no actions, one action, or several actions. Use only the actions listed below. If the goal is already achieved in the world state, return an empty sequence of actions. Use the fewest actions to generate the plan. You have to response in JSON format.\n\n"
        self._grammar = json.dumps({
                "type": "object",
                "properties": {
                    "reasoning": {
                        "type": "string"
                    },
                    "actions": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "action_name": {
                                    "type": "string"
                                },
                                "action_args": {
                                    "type": "string"
                                }
                            }
                        },
                        "minItems": 0,
                        "maxItems": -1
                    }
                },
            })

        self._cb_group = ReentrantCallbackGroup()
        self._srv = self.create_service(ExecutePlan, 'gpsr_planning', callback=self._execute_cb, callback_group=self._cb_group)
        self._action_client = ActionClient(self, GenerateResponse, "/llama/generate_response", callback_group=self._cb_group)
        
    def _execute_cb(self, request, result):
        self.get_logger().info(f'Executing plan with the following command: {request.command}')

        self._prompt = self._prompt + "ACTIONS:\n" + self._bt_nodes + "\n\n" + "GOAL:"+ request.command + "\n\n" + "### Instruction:\nGenerate a plan to achieve the goal.\n\n### Response:\n"
        
        self.get_logger().info(f'prompt: {self._prompt}')

        goal_msg = GenerateResponse.Goal()
        goal_msg.prompt = self._prompt
        goal_msg.reset = True
        goal_msg.sampling_config.temp = 0
        goal_msg.sampling_config.grammar = self._grammar

        self._action_client.wait_for_server()

        self._action_client.send_goal_async(goal_msg)

        result.success = True

        return result

def main(args=None):
    rclpy.init(args=args)
    node = GPSRPlanning()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    executor.spin()
    executor.shutdown()


if __name__ == "__main__":
    main()