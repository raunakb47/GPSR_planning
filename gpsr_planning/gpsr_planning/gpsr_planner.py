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
from llama_ros.langchain import LlamaROS
from langchain.prompts import PromptTemplate
from langchain_core.output_parsers import StrOutputParser
import datetime as dt


class GpsrPlanner:

    def __init__(
        self,
        robot_actions_path: str = "robot_actions.json",
        waypoints_path: str = "waypoints.json"
    ) -> None:

        self.robot_actions = json.load(open(robot_actions_path))
        self.waypoints_path = waypoints_path

        self.create_grammar()
        self.load_waypoints()

        self.llm = LlamaROS(
            temp=0.40,
            grammar_schema=self.grammar_schema
        )

        time_h = dt.datetime.now().strftime("%H:%M")
        tomorrow = (dt.datetime.now() + dt.timedelta(days=1)).strftime("%A")
        day = dt.datetime.now().strftime("%A")
        
        print(time_h)

        # create a prompt template
        prompt_template = (
            "You are a robot named Tiago who is participating in the Robocup with the Gentlebots team from Spain, made up of the Rey Juan Carlos University of Madrid and the University of León. "
            "You have to generate plans, sequence of actions, to achive goals. "
            # "A plan is a sequence of actions. "
            # "Only use one action at a time. "
            "Use the least number of actions as possible. "
            "Use only the actions listed below and try to speak as much as you can. "
            # "Some actions require arguments. If they are empty, unknown or they are not explicit defined, answer unknown. "
            "Some action arguments may be unknown, if so, answer unknown. "
            "Today is " + day +", tomorrow is " + tomorrow + " and the time is " + time_h + ". "
            "You start at the instruction point. "
            "Use the move_to action before each action that requires changing the waypoint. "
            # "You know the waypoint of all rooms, furniture, tables and in the house. Do not need to find these waypoint. "
            # "Remember you can answer questions with the action answer_quiz."
            "\n\n"

            "ACTIONS:\n"
            "{actions_descriptions}\n\n"

            "### Instruction:\n"
            "You are at the instruction point, generate a plan to achieve your goal: {prompt}\n\n"

            "### Response:\n"
        )

        prompt = PromptTemplate(
            input_variables=["actions_descriptions", "prompt"],
            template=prompt_template
        )

        # create a chain with the llm and the prompt template
        self.chain = prompt | self.llm | StrOutputParser()

    def cancel(self) -> None:
        self.llm.cancel()

    def send_prompt(self, prompt: str) -> Tuple[dict | str]:

        prompt = prompt + " "
        prompt = prompt.replace(
            " me ", " to the person in the instruction point ").replace("to to", "to")
        prompt = prompt.replace("them", "him")
        prompt = prompt.strip()

        response = self.chain.invoke({
            "prompt": prompt,
            "actions_descriptions": self.actions_descriptions[:-1],
        })

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
        for a in self.robot_actions:
            self.actions_descriptions += f"- {a['name']}: {a['description']}\n"
            actions_refs.append({"$ref": f"#/definitions/{a['name']}"})

        action_definitions = {}
        for a in self.robot_actions:

            required = [arg for arg in a["args"]]
            properties = {}

            for arg in a["args"]:
                properties[arg] = {"type": a["args"][arg]["type"]}

                if "choices" in a["args"][arg]:
                    properties[arg]["enum"] = a["args"][arg]["choices"]

            action_definitions[a["name"]] = {
                "type": "object",
                "properties": {
                    "explaination_of_next_actions": {
                        "type": "string"
                    },
                    a["name"]: {
                        "type": "object",
                        "properties": properties,
                        "required": required
                    }
                },
                "required": [a["name"], "explaination_of_next_actions"]
            }

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
                    "maxItems": 100
                },
            },
            "required": ["actions"]
        })
