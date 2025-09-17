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
        nfr_profiles_path: str = "src/GPSR_planning/gpsr_planning/params/nfr_profiles.json"
    ) -> None:

        self.robot_actions = json.load(open(robot_actions_path))
        self.waypoints_path = waypoints_path
        self.objects_path = objects_path
        self.names_path = names_path

        # Load NFR profiles
        self.nfr_profiles = json.load(open(nfr_profiles_path))["profiles"]

        # Load summaries for prompt (concise)
        self.objects_summary = "Standard YCB objects (e.g., apple, banana, cup; full list per RoboCup@Home rules)"
        self.categories_summary = "Standard categories (e.g., food, tools; full list per rules)"
        self.names_summary = "Common names (e.g., Alice, Bob; full list per rules)"
        self.waypoints_summary = "Arena waypoints (e.g., kitchen, table; full list per rules)"

        self.create_grammar()

        # Initialize LLM
        self.llm = ChatLlamaROS(
            temp=0.10,  # Lowered from 0.30 for more determinism
            grammar_schema=self.grammar_schema
        )

        is_lora_added = False

        # Concise actions_descriptions
        self.actions_descriptions = ""
        for robot_act in self.robot_actions:
            self.actions_descriptions += f"- {robot_act['name']}: {robot_act['description']}\n"

        # First chain: Generate base functional plan 
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

        # Second chain: Refine chain prompt (strengthened for strict JSON output)
        refine_prompt_template = ChatPromptTemplate.from_messages([
            SystemMessagePromptTemplate.from_template(
                "Refine the base plan by incorporating NFR constraints only if they suggest adding improvements like checks or fallbacks. "
                "Do not change existing actions, parameters, or explanations from the base plan unless absolutely necessary for NFR compliance. "
                "Keep explanations clear, concise, and in plain English—avoid any special characters, numbers, or nonsense. "
                "If no relevant NFR or no improvements needed, output the base plan unchanged. "

                + ("The output should be a JSON object with a key 'actions' containing a list of actions. "
                   "Each action has 'explanation_of_next_actions', explaining the reason for the action (including NFR reasoning if applicable), "
                   "and an action-specific key (e.g., find_object) with its parameters. "
                   "Only output the JSON object without any additional explanatory text or steps. " if not is_lora_added else '')

                + "\nExample 1 (No refinement needed):\n"
                "Base plan: {{\"actions\": [{{\"explanation_of_next_actions\": \"Move to kitchen.\", \"move_to\": {{\"destination\": \"kitchen\"}}}}]}}\n"
                "NFR: - move_to: Quality: Accurate navigation.\n"
                "Refined output: {{\"actions\": [{{\"explanation_of_next_actions\": \"Move to kitchen.\", \"move_to\": {{\"destination\": \"kitchen\"}}}}]}}\n\n"

                + "Example 2 (Add fallback for grasp per NFR):\n"
                "Base plan: {{\"actions\": [{{\"explanation_of_next_actions\": \"Pick object.\", \"pick_object\": {{\"object\": \"apple\"}}}}]}}\n"
                "NFR: - pick_object: Quality: Secure grasping; Add fallback if grasp fails.\n"
                "Refined output: {{\"actions\": [{{\"explanation_of_next_actions\": \"Pick object, with NFR fallback for grasp failure.\", \"pick_object\": {{\"object\": \"apple\"}}}}, {{\"explanation_of_next_actions\": \"Fallback if grasp fails (NFR compliance).\", \"speak\": {{\"say_text\": \"Grasp failed, retrying.\"}}}}]}}\n"

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
        print("Base Response:", base_response)  # For debugging

        try:
            base_plan = json.loads(base_response)
        except json.JSONDecodeError as e:
            print(f"Base plan parse error: {e} - Using empty plan as fallback")
            base_plan = {"actions": []}  # Fallback to avoid crash

        # Step 2: Extract unique action names from base plan
        action_names = set()
        for action in base_plan.get("actions", []):
            for key in action:
                if key != "explanation_of_next_actions":
                    action_names.add(key)

        # Step 3: Build filtered NFR summary only for relevant actions
        filtered_nfr_summary = "NFR PROFILES (apply only these relevant constraints to the actions in the base plan):\n"
        has_relevant_nfr = False
        for profile in self.nfr_profiles:
            if profile['name'] in action_names:
                has_relevant_nfr = True
                filtered_nfr_summary += (
                    f"- {profile['name']}: Quality: {', '.join(profile['Quality Attribute(s)'])}; "
                    f"Robot Constraints: {', '.join(profile['Robot Constraints'])}; "
                    f"Operational Constraints: {', '.join(profile['Operational Constraints'])}\n"
                )
        if not has_relevant_nfr:
            print("No relevant NFR profiles—skipping refinement.")
            return base_plan, prompt  # Bypass refinement entirely if no relevant NFR

        # Step 4: Refine only if relevant NFR exists
        refined_response = self.refine_chain.invoke({
            "base_plan": json.dumps(base_plan),
            "prompt": prompt,
            "nfr_summary": filtered_nfr_summary
        })
        print("Refined Response:", refined_response)  # For debugging

        try:
            refined_plan = json.loads(refined_response)
            # Step 5: Validate refined plan (check for garbage or major changes)
            if self._is_valid_refinement(base_plan, refined_plan):
                return refined_plan, prompt
            else:
                print("Invalid or garbage refinement—falling back to base plan.")
                return base_plan, prompt
        except json.JSONDecodeError as e:
            print(f"Refine parse error: {e} - Falling back to base plan")
            return base_plan, prompt

    def _is_valid_refinement(self, base_plan, refined_plan):
        """Simple validation: Check if actions are similar and no garbage in explanations."""
        base_actions = [list(a.keys())[1] for a in base_plan.get('actions', []) if len(a) > 1]  # Action types
        refined_actions = [list(a.keys())[1] for a in refined_plan.get('actions', []) if len(a) > 1]
        
        # Allow additions but check for complete mismatch
        if set(base_actions) - set(refined_actions):  # Core base actions missing
            return False
        
        # Check explanations for garbage (e.g., special chars, <unk>, short nonsense)
        for action in refined_plan.get('actions', []):
            exp = action.get('explanation_of_next_actions', '')
            if len(exp) < 10 or re.search(r'<unk>|<s>|[\$#&@%*]{3,}', exp) or len(re.findall(r'[^a-zA-Z0-9\s\.,\?!]', exp)) / max(1, len(exp)) > 0.3:
                return False
        
        return True

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
                    "maxItems": 10
                },
            },
            "required": ["actions"]
        })