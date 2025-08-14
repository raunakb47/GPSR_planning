import json
from typing import Tuple
from llama_ros.langchain import ChatLlamaROS
from langchain_core.prompts import ChatPromptTemplate, SystemMessagePromptTemplate, HumanMessagePromptTemplate
from langchain_core.output_parsers import StrOutputParser
import datetime as dt
import re
from itertools import product

class GpsrPlanner:

    def __init__(
        self,
        robot_actions_path: str = "robot_actions.json",
        waypoints_path: str = "waypoints.json",
        objects_path: str = "objects.json",
        names_path: str = "names.json",
        nfr_profiles_path: str = "nfr_profiles.json"
    ) -> None:

        self.robot_actions = json.load(open(robot_actions_path))
        self.waypoints_path = waypoints_path
        self.objects_path = objects_path
        self.names_path = names_path

        # Load NFR profiles
        self.nfr_profiles = json.load(open(nfr_profiles_path))["profiles"]

        # Load allowed entities for prompt constraints
        with open(self.objects_path, 'r') as f:
            obj_json = json.load(f)
            self.objects = sorted([o.replace(" ", "_") for o in obj_json['items']])
            self.categories = sorted([c.replace(" ", "_") for c in obj_json['categories']])

        with open(self.names_path, 'r') as f:
            names_json = json.load(f)
            self.names = sorted(names_json['names'])

        with open(self.waypoints_path, 'r') as f:
            wp_json = json.load(f)
            self.waypoints = []
            for ele in wp_json:
                self.waypoints.append(ele['room'])
                self.waypoints.extend(ele['locations'])
            self.waypoints = sorted([w.replace(" ", "_") for w in self.waypoints])

        # Build string representations for the prompt
        self.objects_str = ", ".join(self.objects)
        self.categories_str = ", ".join(self.categories)
        self.names_str = ", ".join(self.names)
        self.waypoints_str = ", ".join(self.waypoints)

        self.create_grammar()
        # self.load_waypoints()  # Not needed anymore, as waypoints are loaded above

        # Initialize LLM with lower temperature for less creativity/hallucinations
        self.llm = ChatLlamaROS(
            temp=0.30,  # Reduced from 0.60 to minimize hallucinations
            grammar_schema=self.grammar_schema
        )

        is_lora_added = False

        # Build enhanced actions_descriptions with NFR constraints for all actions
        self.actions_descriptions_with_nfr = ""
        for robot_act in self.robot_actions:
            # Find matching NFR profile
            profile = next((p for p in self.nfr_profiles if p["name"] == robot_act["name"]), None)
            if profile:
                nfr_info = (
                    f" (Quality Attributes: {', '.join(profile['Quality Attribute(s)'])}; "
                    f"Robot Constraints: {', '.join(profile['Robot Constraints'])}; "
                    f"Operational Constraints: {', '.join(profile['Operational Constraints'])})"
                )
            else:
                nfr_info = " (No NFR profile available)"
            self.actions_descriptions_with_nfr += f"- {robot_act['name']}: {robot_act['description']}{nfr_info}\n"

        chat_prompt_template = ChatPromptTemplate.from_messages([
            SystemMessagePromptTemplate.from_template(
                "You are a robot named Tiago participating in the Robocup with the Gentlebots team from Spain, "
                "made up of the Rey Juan Carlos University of Madrid and the University of León. "

                + ("You have to generate plans, sequence of actions, to achieve goals. "
                   "Use only the actions listed below, respecting their associated constraints. "
                   "Do not invent new actions, objects, categories, names, or waypoints. "
                   "For each action, ensure it respects the Quality Attributes, Robot Constraints, and Operational Constraints. "
                   "If necessary, add additional actions (e.g., checks, fallbacks, or verifications) to satisfy the NFR constraints without inventing new elements. " if not is_lora_added else "")

                + ("The output should be a JSON object with a key 'actions' containing a list of actions. "
                   "Each action has 'explanation_of_next_actions', explaining the reason for the action (including how it addresses NFR if applicable), "
                   "and an action-specific key (e.g., find_object) with its parameters. "
                   "Only output the JSON object without any additional explanatory text or steps. "
                if not is_lora_added else '')

                + "Actions are performed at waypoints, so move to the waypoints to perform actions. "
                "Rooms, furniture, and tables are considered as waypoints and there is no need to find them. "
                "Today is {day}, tomorrow is {tomorrow} and the time is {time_h}. "
                "\n\n"

                + ("ACTIONS:\n"
                "{actions_descriptions}" if not is_lora_added else '')

                + "\nAllowed items (objects): {objects_str}\n"
                "Allowed categories: {categories_str}\n"
                "Allowed names: {names_str}\n"
                "Allowed waypoints (locations): {waypoints_str}\n"
                "When choosing parameters for actions, ONLY use values from these allowed lists above. "
                "Stick strictly to the RoboCup@Home environment and Tiago's capabilities."
            ),
            HumanMessagePromptTemplate.from_template(
                "You are at the instruction point, generate a plan to achieve your goal: {prompt}"
            )
        ])

        # create a chain with the llm and the prompt template
        self.chain = chat_prompt_template | self.llm | StrOutputParser()

    def cancel(self) -> None:
        self.llm.cancel()

    def send_prompt(self, prompt: str) -> Tuple[dict | str]:

        prompt = prompt + " "
        prompt = re.sub(r'\b(?:tell|say)\s+me\b', lambda match: match.group(0).replace("me", "at the instruction point"), prompt, flags=re.IGNORECASE)
        prompt = prompt.replace("to to", "to").replace("them", "him").strip()

        today_dt = dt.datetime.now()
        day_suffix = lambda day: 'th' if 11 <= day <= 13 else {1: 'st', 2: 'nd', 3: 'rd'}.get(day % 10, 'th')
        day = today_dt.strftime(f"%A {today_dt.day}{day_suffix(today_dt.day)}")

        time_h = today_dt.strftime("%H:%M")
        tomorrow = (today_dt + dt.timedelta(days=1)).strftime("%A")

        response = self.chain.invoke({
            "prompt": prompt,
            "actions_descriptions": self.actions_descriptions_with_nfr[:-1],  # Use NFR-enhanced descriptions
            "objects_str": self.objects_str,
            "categories_str": self.categories_str,
            "names_str": self.names_str,
            "waypoints_str": self.waypoints_str,
            "day": day,
            "tomorrow": tomorrow,
            "time_h": time_h
        })

        print(response)

        return json.loads(response), prompt

    def load_waypoints(self) -> None:
        # Deprecated, as waypoints are loaded in __init__
        pass

    def create_grammar(self) -> None:
        self.actions_descriptions = ""
        actions_refs = []
        for robot_act in self.robot_actions:
            self.actions_descriptions += f"- {robot_act['name']}: {robot_act['description']}\n"
            actions_refs.append({"$ref": f"#/definitions/{robot_act['name']}"})

        action_definitions = {}
        for robot_act in self.robot_actions:
            # Always define action_args base
            action_args = {
                "type": "object",
                "properties": {},
                "required": []
            }

            if robot_act['arg_case'] == 'allOf' and len(robot_act['args'].keys()) != 0:
                for arg in robot_act["args"]:
                    action_args['properties'][arg] = {"type": robot_act["args"][arg]["type"]}
                    action_args['required'].append(arg)
                    if "choices" in robot_act["args"][arg]:
                        action_args["properties"][arg]["enum"] = robot_act["args"][arg]["choices"]

            elif robot_act['arg_case'] == 'anyOf' and robot_act['name'] == 'find_object':
                action_args['oneOf'] = []

                item_list = {'item': ['category', 'specific_item'], 'category': ['category'], 'none': []}
                size_list = ['size', 'weight']

                args_obj = {}
                for arg in robot_act["args"]:
                    args_obj[arg] = {"type": robot_act["args"][arg]["type"], "enum": robot_act["args"][arg]["choices"]}

                for item, size in product(list(item_list.keys()), size_list):
                    action_args['oneOf'].append({
                        "properties": {k: args_obj[k] for k in [*item_list[item], size]},
                        "required": item_list[item]
                    })

            elif robot_act['arg_case'] == 'anyOf':
                for arg in robot_act["args"]:
                    action_args['properties'][arg] = {"type": robot_act["args"][arg]["type"]}
                    if "choices" in robot_act["args"][arg]:
                        action_args["properties"][arg]["enum"] = robot_act["args"][arg]["choices"]

            elif robot_act['arg_case'] == 'oneOf':
                action_args['oneOf'] = []
                del action_args['properties']
                del action_args['required']

                for search_by_option in robot_act['args']['search_by']['choices']:
                    option_obj = {
                        "properties": {
                            "search_by": {
                                "const": search_by_option
                            }
                        },
                        "required": ['search_by']
                    }

                    if search_by_option != 'none':
                        option_obj['properties'][search_by_option] = {
                            'type': robot_act['args'][search_by_option]['type'],
                            'enum': robot_act['args'][search_by_option]['choices']
                        }

                    # idk if we should let this here
                    if 'previously_found' in robot_act['args']:
                        # option_obj["required"].append('previously_found')
                        option_obj["properties"]['previously_found'] = {
                            'type': 'boolean'
                        }
                        option_obj['required'].append(search_by_option)

                    action_args['oneOf'].append(option_obj)

            action_def = {
                "type": "object",
                "properties": {
                    "explanation_of_next_actions": {
                        "type": "string",
                        "maxLength": 200
                    },
                    robot_act['name']: action_args
                },
                "required": ['explanation_of_next_actions', robot_act['name']]
            }

            action_definitions[robot_act["name"]] = action_def

        self.grammar_schema = json.dumps({
            "definitions": action_definitions,
            "type": "object",
            "properties": {
                "actions": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "anyOf": actions_refs,
                    },
                    "minItems": 1,
                    "maxItems": 15
                },
            },
            "required": ["actions"]
        })