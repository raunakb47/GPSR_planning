#!/usr/bin/env python3

# MIT License

# Copyright (c) 2023  Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import json
from typing import Tuple
from llama_ros.langchain import ChatLlamaROS
from langchain_core.prompts import ChatPromptTemplate, SystemMessagePromptTemplate, HumanMessagePromptTemplate
from langchain_core.output_parsers import StrOutputParser
import datetime as dt
import re
from itertools import product

class GpsrPlanner:

    def __init__(self,
                robot_actions_path: str = "robot_actions.json",
                waypoints_path: str = "waypoints.json", 
                nfr_profiles_path: str = "nfr_profiles.json",   # Added NFR profiles path
                ) -> None:
        
        self.robot_actions = json.load(open(robot_actions_path))
        self.waypoints_path = waypoints_path

        # Load NFR profiles and select the specified one
        self.nfr_profiles = json.load(open(nfr_profiles_path))["profiles"]
        
        self.create_grammar()
        # self.load_waypoints()

        # Initialize LLM
        self.llm = ChatLlamaROS(
            temp=0.60,
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
                "Use only the actions listed below. "  if not is_lora_added else "")
                
                + ("The output should be a JSON object with a key 'actions' containing a list of actions. "
                   "Each action has 'explanation_of_next_actions', explaining the reason for the action, "
                   "and an action-specific key (e.g., find_object) with its parameters. "
                   "Only output the JSON object without any additional explanatory text or steps. " 
                if not is_lora_added else '') +

                # theoretically better but worse results
                # "The format of the output of the plan should be {{explanation_of_next_actions, action}}[], "
                # "where explanation_of_next_actions is a string with an explanation on why you choose the action, the action object key is the action name and the value is an object with the action parameters."

                "Actions are performed at waypoints, so move to the waypoints to perform actions. "
                "Rooms, furniture, and tables are considered as waypoints and there is no need to find them. "
                # "Use the move_to action before each action that requires changing the waypoint and remember your current waypoint. "
                # "Answer only to the arguments you are asked for. "
                "Today is {day}, tomorrow is {tomorrow} and the time is {time_h}. "
                # "You start at the instruction point. "
                "\n\n"

                + ("ACTIONS:\n"
                "{actions_descriptions}" if not is_lora_added else '')
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
            "day": day,
            "tomorrow": tomorrow,
            "time_h": time_h
        })
        
        print(response)

        return json.loads(response), prompt

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
            if robot_act["name"] == self.selected_profile["name"]:
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
