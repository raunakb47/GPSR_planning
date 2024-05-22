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
            temp=0.0,
            grammar_schema=self.grammar_schema
        )

        # create a prompt template
        prompt_template = (
            "You are a robot named Tiago from the Gentlebots team. "
            "You have to generate plans to achive goals. "
            "A plan is a sequence of actions. "
            "Use only the actions listed below and use the less action as you can. "
            "Use the move action before each action that requires changing the location.\n\n"

            "WAYPOINTS:\n"
            "{waypoints}\n\n"

            "ACTIONS:\n"
            "{actions_descriptions}\n\n"

            "### Instruction:\n"
            "You are at the instruction point, generate a plan to achieve your goal: {prompt}\n\n"

            "### Response:\n"
        )

        prompt = PromptTemplate(
            input_variables=["actions_descriptions", "prompt", "waypoints"],
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
            "waypoints": self.waypoints[:-1]
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
                properties[arg] = {"type": a["args"][arg]}

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
