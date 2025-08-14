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

        # Load allowed entities for grammar (already used), but skip full lists in prompt to reduce length
        # Summaries for prompt if needed
        self.objects_summary = "Standard YCB objects (e.g., apple, banana, cup; full list per RoboCup@Home rules)"
        self.categories_summary = "Standard categories (e.g., food, tools; full list per rules)"
        self.names_summary = "Common names (e.g., Alice, Bob; full list per rules)"
        self.waypoints_summary = "Arena waypoints (e.g., kitchen, table; full list per rules)"

        self.create_grammar()

        # Initialize LLM with low temperature
        self.llm = ChatLlamaROS(
            temp=0.30,
            grammar_schema=self.grammar_schema
        )

        is_lora_added = False

        # Build concise actions_descriptions (without full NFR appended to each)
        self.actions_descriptions = ""
        for robot_act in self.robot_actions:
            self.actions_descriptions += f"- {robot_act['name']}: {robot_act['description']}\n"

        # Build concise NFR summary section
        self.nfr_summary = "NFR PROFILES (apply relevant constraints to actions):\n"
        for profile in self.nfr_profiles:
            self.nfr_summary += (
                f"- {profile['name']}: Quality: {', '.join(profile['Quality Attribute(s)'])}; "
                f"Robot Constraints: {', '.join(profile['Robot Constraints'])}; "
                f"Operational Constraints: {', '.join(profile['Operational Constraints'])}\n"
            )

        # First chain: Generate base functional plan (shorter prompt, no full NFR or lists)
        base_prompt_template = ChatPromptTemplate.from_messages([
            SystemMessagePromptTemplate.from_template(
                "You are a robot named Tiago participating in the Robocup with the Gentlebots team from Spain, "
                "made up of the Rey Juan Carlos University of Madrid and the University of León. "

                + ("You have to generate a base plan: sequence of actions to achieve goals. "
                   "Use only the actions listed below. Do not invent new actions or parameters. "
                   "Parameters must strictly match the robot's capabilities and environment (e.g., objects, waypoints). " if not is_lora_added else "")

                + ("The output should be a JSON object with a key 'actions' containing a list of actions. "
                   "Each action has 'explanation_of_next_actions', explaining the reason for the action, "
                   "and an action-specific key (e.g., find_object) with its parameters. "
                   "Only output the JSON object without any additional text. "
                if not is_lora_added else '')

                + "Actions are performed at waypoints, so move to waypoints as needed. "
                "Rooms, furniture, and tables are waypoints (no need to find them). "
                "Today is {day}, tomorrow is {tomorrow} and the time is {time_h}. "
                "\n\n"

                + ("ACTIONS:\n"
                "{actions_descriptions}" if not is_lora_added else '')

                + "\nStick to allowed parameters: Objects: {objects_summary}; Categories: {categories_summary}; "
                "Names: {names_summary}; Waypoints: {waypoints_summary}. "
                "The grammar enforces valid choices—do not deviate."
            ),
            HumanMessagePromptTemplate.from_template(
                "You are at the instruction point, generate a base plan for: {prompt}"
            )
        ])

        self.base_chain = base_prompt_template | self.llm | StrOutputParser()

        # Second chain: Refine with NFR (input base plan, apply constraints)
        refine_prompt_template = ChatPromptTemplate.from_messages([
            SystemMessagePromptTemplate.from_template(
                "Refine the base plan by incorporating NFR constraints. "
                "Add checks, fallbacks, or verifications as needed to satisfy NFR without inventing new elements. "
                "Use only existing actions and valid parameters. "

                + ("Output the refined JSON plan in the same format. "
                   "Update explanations to include NFR reasoning if applicable. " if not is_lora_added else '')

                + "\n{nfr_summary}"
            ),
            HumanMessagePromptTemplate.from_template(
                "Base plan: {base_plan}\nRefine for goal: {prompt}"
            )
        ])

        self.refine_chain = refine_prompt_template | self.llm | StrOutputParser()

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

        # Step 1: Generate base plan
        base_response = self.base_chain.invoke({
            "prompt": prompt,
            "actions_descriptions": self.actions_descriptions[:-1],
            "objects_summary": self.objects_summary,
            "categories_summary": self.categories_summary,
            "names_summary": self.names_summary,
            "waypoints_summary": self.waypoints_summary,
            "day": day,
            "tomorrow": tomorrow,
            "time_h": time_h
        })

        base_plan = json.loads(base_response)

        # Step 2: Refine with NFR
        refined_response = self.refine_chain.invoke({
            "base_plan": json.dumps(base_plan),
            "prompt": prompt,
            "nfr_summary": self.nfr_summary
        })

        print(refined_response)

        return json.loads(refined_response), prompt

    def create_grammar(self) -> None:
        actions_refs = []
        for robot_act in self.robot_actions:
            actions_refs.append({"$ref": f"#/definitions/{robot_act['name']}"})

        action_definitions = {}
        for robot_act in self.robot_actions:
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

                    if 'previously_found' in robot_act['args']:
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