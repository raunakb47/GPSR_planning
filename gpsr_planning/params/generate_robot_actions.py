import json
import os

file_path = os.path.dirname(os.path.realpath(__file__))

template_file = os.path.join(file_path, "robot_actions_template.json")
robot_actions_file = os.path.join(file_path, "robot_actions.json")
objects_file = os.path.join(file_path, "objects.json")
names_file = os.path.join(file_path, "names.json")
waypoints_file = os.path.join(file_path, "waypoints.json")

waypoints = []
with open(waypoints_file, 'r') as f:
    wp_json = json.load(f)
    for wp in wp_json:
        waypoints.append(wp)
        
waypoints = sorted(waypoints)
waypoints = [w.replace(" ", "_") for w in waypoints]

objects = []
classes = []
with open(objects_file, 'r') as f:
    obj_json = json.load(f)
    objects = obj_json['items']
    classes = obj_json['categories']

objects = sorted(objects)
objects = [o.replace(" ", "_") for o in objects]
classes = sorted(classes)
classes = [c.replace(" ", "_") for c in classes]

names = []
with open(names_file, 'r') as f:
    names_json = json.load(f)
    names = names_json['names']
    
names = sorted(names)

with open(template_file, 'r') as f:
    template_json = json.load(f)
    actions = template_json

    for action in actions:
        args = action['args']
        for arg in args.values():
            if 'choices' in arg:
                type = arg['choices']
                if type == '<items>':
                    arg['choices'] = objects
                elif type == '<categories>':
                    arg['choices'] = classes
                elif type == '<names>':
                    arg['choices'] = names
                elif type == '<waypoints>':
                    arg['choices'] = waypoints
    
    with open(robot_actions_file, 'w') as f:
        json.dump(actions, f, indent=4)


print("Generated robot_actions.json")