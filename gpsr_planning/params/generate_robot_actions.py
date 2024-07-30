import json
import os
import re

file_path = os.path.dirname(os.path.realpath(__file__))

template_file = os.path.join(file_path, "robot_actions_template.json")
objects_file = os.path.join(file_path, "objects.json")
names_file = os.path.join(file_path, "names.json")
waypoints_file = os.path.join(file_path, "waypoints.json")

waypoints = []
with open(waypoints_file, 'r') as f:
    wp_json = json.load(f)
    for wp in wp_json:
        waypoints.append(wp['room'])
        for l in wp['locations']:
            waypoints.append(l)

objects = []
classes = []
with open(objects_file, 'r') as f:
    obj_json = json.load(f)
    objects = obj_json['items']
    classes = obj_json['categories']

names = []
with open(names_file, 'r') as f:
    names_json = json.load(f)
    names = names_json['names']

with open(template_file, 'r') as f:
    template_json = json.load(f)
    actions = template_json

    for action in actions:
        args = action['args']
        for arg in args.values():
            if 'choices' in arg:
                type = arg['choices']
                if type == '<objects>':
                    arg['choices'] = objects
                elif type == '<object_classes>':
                    arg['choices'] = classes
                elif type == '<names>':
                    arg['choices'] = names
                elif type == '<waypoints>':
                    arg['choices'] = waypoints
    
    with open('robot_actions.json', 'w') as f:
        json.dump(actions, f, indent=4)


print("Generated robot_actions.json")