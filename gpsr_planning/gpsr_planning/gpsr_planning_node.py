import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from gpsr_msgs.srv import ExecutePlan
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from llama_msgs.action import GenerateResponse
from xml.dom import minidom

class GPSRPlanning(Node):

    def __init__(self):
        super().__init__("planning_node")

        count_bt_node = """count: count the objects in the room, args json schema: {'object_type': 'Type of the object to be counted'}"""

        describeO_bt_node = """describe_object: describe an object, args json schema: {'object': 'Object to be described'}"""

        describeP_node = """describe_person: describe a person, args json schema: {'person': 'Person to be described'}"""

        findO_bt_node = """find_object: find an object in the room, args json schema: {'object': 'Object to be found'}"""

        findP_bt_node = """find_person: find a person in the room, args json schema: {'person_description': 'Description of the person to find'}"""

        followP_bt_node = """follow_person: the robot constantly follows the person staying behind them all the times, args json schema: {'person_name': 'Name of the person'}"""

        listen_bt_node = """listen: listen and transcribe what a person tells to the robot, args json schema: { }"""

        lookAround_bt_node = """look_around: turn its head and look around for something, args json schema: { }"""

        lookTo_bt_node = """look_to: make the robot look at the person, args json schema: {'person_name': 'Name of the person'}"""

        moveTo_bt_node = """move_to: the robot moves from its current location to a specific point to perform a task or to approach an object or person to interact with, args json schema: {'destination': 'Waypoint destination'}"""

        guide_bt_node = """guide_person: robot take or guide a person to a specific destination, continuosly moving towards the location where the guided person has to be, args json schema: {'destination': 'Target Waypoint', 'person_description': 'Description of the person to guide'}"""

        offer_bt_node = """offer: offer an object to a person who is at the same waypoint as the robot, args json schema: {'object': 'Object to be offered'}"""

        pickO_bt_node = """pick_object: pick and object in the location, args json schema: {'object': 'Object to be picked'}"""

        placeO_bt_node = """place_object: place and object in the location, args json schema: {'object': 'Object to be placed'}"""

        query_bt_node = """query_object: obtain information about what a person has told it, args json schema: {'text': 'Text to be processed'}"""

        recognizeP_bt_node = """recognize_person: recognize a person after having met her, args json schema: {person': 'Person to be recognized'}"""

        speak_bt_node = """speak: speak something to a person, args json schema: {'text': 'Text to tell the person'}"""

        bt_node_list = [
            count_bt_node,
            describeO_bt_node,
            describeP_node,
            followP_bt_node,
            findO_bt_node,
            findP_bt_node,
            listen_bt_node,
            guide_bt_node,
            lookAround_bt_node,
            lookTo_bt_node,
            moveTo_bt_node,
            offer_bt_node,
            pickO_bt_node,
            placeO_bt_node,
            query_bt_node,
            recognizeP_bt_node,
            speak_bt_node
        ]

        self._bt_nodes = ""
        for bt in bt_node_list:
            self._bt_nodes += f"- {bt}\n"

        self._prompt = """You are an AI planner for a robotic assistant named TIAGo. 
You have to generate plans, which are sequences of actions to achieve a goal. 
Plans may contain no actions, one action, or several actions. 
Use only the actions listed below. 
Make plans as short as you can. 
If the goal is already achieved in the world state, return an empty sequence of actions. 
Reason about the goal, the plan and the actions. 
You have to response in JSON format.\n\n"""

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
                                    "type": "array",
                                    "items": {
                                        "type": "object",
                                        "properties":{
                                            "arg_name": {
                                                "type": "string"
                                            },
                                            "arg_value": {
                                                "type": "string"
                                            }
                                        }
                                    }
                                }
                            }
                        },
                        "minItems": 0,
                        "maxItems": -1
                }
            }
        })

        self._cb_group = ReentrantCallbackGroup()
        self._srv = self.create_service(
            ExecutePlan, "gpsr_planning", callback=self._execute_cb, callback_group=ReentrantCallbackGroup())
        self._action_client = ActionClient(
            self, GenerateResponse, "/llama/generate_response", callback_group=ReentrantCallbackGroup())
        self._action_client.wait_for_server()

    def _execute_cb(self, request: ExecutePlan.Request, response: ExecutePlan.Response) -> ExecutePlan.Response:
        self.get_logger().info(
            f"Executing plan with the following command: {request.command}")

        prompt = self._prompt + "ACTIONS:\n" + self._bt_nodes + "\n\n" + "GOAL: " + request.command + \
            "\n\n" + "### Instruction: Generate a plan to fully achieve the goal.\n### Response:"

        self.get_logger().info(f"prompt: {prompt}")

        goal_msg = GenerateResponse.Goal()
        goal_msg.prompt = prompt
        goal_msg.reset = True
        goal_msg.sampling_config.temp = 0.0
        goal_msg.sampling_config.grammar_schema = self._grammar
        goal_msg.sampling_config.prop_order = [
            "reasoning", "actions", "action_name", "action_args"]

        self.get_logger().info("planning...")

        result: GenerateResponse.Result = self._action_client.send_goal(
            goal_msg).result
        self.get_logger().info(f"{result.response.text}")

        

        self.get_logger().info("finished planning")

        response.bt_xml = self.action_parser(result.response.text)

        return response
    
    def action_parser(self, response_plan):


        plan = json.loads(response_plan)

        self.get_logger().info(f'{plan.get("actions")}')

        bt_xml = minidom.Document()
        

        root_element = bt_xml.createElement('root')
        root_element.setAttribute('main_tree_to_execute', 'BehaviorTree')
        bt_element = bt_xml.createElement('BehaviorTree')
        bt_element.setAttribute('ID', 'BehaviorTree')
        sequence_element = bt_xml.createElement('Sequence')

        
        
        bt_nodes = []

        for action in plan.get("actions"):
            self.get_logger().info(f'{action}')
            action_element = bt_xml.createElement('Action')
            action_element.setAttribute('ID', action.get("action_name"))
            self.get_logger().info(f'{action.keys()}')
            self.get_logger().info(f'{action.get("action_name")}')
            self.get_logger().info(f'{action.get("action_args")}')

            
            for arg in action.get("action_args"):
                self.get_logger().info(f'{arg}')
                arg_key = arg.keys()
                self.get_logger().info(f'{arg_key}')
                action_element.setAttribute(arg.get('arg_name'), arg.get('arg_value'))

            sequence_element.appendChild(action_element)

        bt_element.appendChild(sequence_element)
        root_element.appendChild(bt_element)
        bt_xml.appendChild(root_element)
        str_bt = bt_xml.toprettyxml(indent="\t")
        self.get_logger().info(f'{str_bt}')

        return str_bt


def main(args=None):
    rclpy.init(args=args)
    node = GPSRPlanning()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    executor.shutdown()


if __name__ == "__main__":
    main()
