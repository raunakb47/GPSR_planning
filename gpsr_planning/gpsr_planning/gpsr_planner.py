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
        # objects_path: str = "objects.json",
        # names_path: str = "names.json",
        nfr_profiles_path: str = "src/GPSR_planning/gpsr_planning/params/nfr_profiles.json"
    ) -> None:

        self.robot_actions = json.load(open(robot_actions_path))
        self.waypoints_path = waypoints_path

        self.create_grammar()
        # self.load_waypoints()

        self.llm = ChatLlamaROS(
            temp=0.20,
            grammar_schema=self.grammar_schema
        )
        
        is_lora_added = False

        # Base chain prompt (Based on original for consistent functional plan)
        base_prompt_template = ChatPromptTemplate.from_messages([
            SystemMessagePromptTemplate.from_template(
                "You are a robot named Tiago who is participating in the Robocup with the Gentlebots team from Spain, "
                "made up of the Rey Juan Carlos University of Madrid and the University of León. "
                
                + ("You have to generate plans, sequence of actions, to achieve goals. "
                "Use only the actions listed below. " if not is_lora_added else "")
                
                + ("The output should be a JSON object with a key 'actions' containing a list of actions. "
                   "Each action has 'explanation_of_next_actions', explaining the reason for the action, "
                   "and an action-specific key (e.g. find_object) with its parameters. "
                   "Only output the JSON object without any additional explanatory text or steps. " 
                if not is_lora_added else '') +

                "Actions are performed at waypoints, so must move to the waypoints to perform actions. "
                "Rooms, furniture and tables are considered as waypoints and there is no need to find them. "
                "Today is {day}, tomorrow is {tomorrow} and the time is {time_h}. "
                "\n\n"

                + ("ACTIONS:\n"
                "{actions_descriptions}" if not is_lora_added else '')
            ),
            HumanMessagePromptTemplate.from_template(
                "You are at the instruction point, generate a plan to achieve your goal: {prompt}"
            )
        ])
        
        # create a chain with the llm and the prompt template
        self.base_chain = base_prompt_template | self.llm | StrOutputParser()

        # Load NFR profiles (added for refinement)
        self.nfr_profiles = json.load(open(nfr_profiles_path))["profiles"]

        # Refine chain prompt (NFR as enhancement, base unchanged unless needed)
        refine_prompt_template = ChatPromptTemplate.from_messages([
            SystemMessagePromptTemplate.from_template(
                "Use NFRs for enhancement of goal actions. Refine the base plan by incorporating NFR constraints only if they are relevant to the actions from base plan. "
                "Do not change existing actions, parameters from the base plan unless absolutely necessary for NFR compliance. "
                "If no relevant NFR or no improvements needed, output the base plan unchanged. "

                + ("The output should be a JSON object with a key 'actions' containing a list of actions. "
                   "Each action has 'explanation_of_next_actions', explaining the reason for the action (including NFR reasoning if applicable), "
                   "and an action-specific key (e.g., find_object) with its parameters. "
                   "Only output the JSON object without any additional explanatory text or steps. " if not is_lora_added else '')

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

        # Step 1: Generate base plan (using original prompt for consistency)
        base_response = self.base_chain.invoke({
            "prompt": prompt,
            "actions_descriptions": self.actions_descriptions[:-1],
            "day": day,
            "tomorrow": tomorrow,
            "time_h": time_h
        })
        
        print(base_response)

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

        # Step 3: Build filtered NFR summary only for relevant actions (concise)
        filtered_nfr_summary = "NFR (add only if improves plan, in few words):\n"
        has_relevant_nfr = False
        for profile in self.nfr_profiles:
            if profile['name'] in action_names:
                has_relevant_nfr = True
                filtered_nfr_summary += f"- {profile['name']}: Ensure {', '.join(profile['Constraints'])}.\n"

        if not has_relevant_nfr:
            print("No relevant NFR profiles—skipping refinement.")
            return base_plan, prompt  # Bypass refinement

        # Step 4: Refine only if relevant NFR exists (use as suggestions, fallback if no change)
        refined_response = self.refine_chain.invoke({
            "base_plan": json.dumps(base_plan),
            "prompt": prompt,
            "nfr_summary": filtered_nfr_summary
        })
        print("Refined Response:", refined_response)  # For debugging

        try:
            refined_plan = json.loads(refined_response)
            if json.dumps(refined_plan) == json.dumps(base_plan):
                print("No changes from refinement—using base plan.")
                return base_plan, prompt
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
        
        # Check explanations for garbage (e.g., special chars, <unk>)
        for action in refined_plan.get('actions', []):
            exp = action.get('explanation_of_next_actions', '')
            if len(exp) < 10 or re.search(r'<unk>|<s>|[\$#&@%*]{3,}', exp) or len(re.findall(r'[^a-zA-Z0-9\s\.,\?!]', exp)) / max(1, len(exp)) > 0.3:
                return False
        
        return True

    def load_waypoints(self) -> None:
        self.waypoints = ""
        waypoints = json.load(open(self.waypoints_path))

        for ele in waypoints:
            self.waypoints += f"- {ele['room']}\n"
            for l in ele["locations"]:
                self.waypoints += f"- {l}\n"

    def create_grammar(self) -> None:
        self.actions_descriptions = ""
        actions_refs = []
        for robot_act in self.robot_actions:
            self.actions_descriptions += f"- {robot_act['name']}: {robot_act['description']}\n"
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