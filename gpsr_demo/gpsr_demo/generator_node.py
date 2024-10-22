#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_path
from os import path
import re
import warnings
import time
from gpsr_msgs.srv import GeneratePlan
from gpsr_demo.CommandGenerator.gpsr_commands import CommandGenerator 
from pynput import keyboard
from threading import Event

class GPSRGenerator(Node):
    def __init__(self):
        super().__init__('gpsr_generator_node')
        
        package_path = get_package_share_path('gpsr_demo')
        
        names_data = self.read_data(path.join(package_path, 'params', 'names.md'))
        locations_data = self.read_data(path.join(package_path, 'params', 'locations.md'))
        objects_data = self.read_data(path.join(package_path, 'params', 'objects.md'))
        rooms_data = self.read_data(path.join(package_path, 'params', 'rooms.md'))
        
        names = self.parse_names(names_data)
        location_names, placement_location_names = self.parse_locations(locations_data)
        object_names, object_categories_plural, object_categories_singular = self.parse_objects(objects_data)
        room_names = self.parse_rooms(rooms_data)
        
        self.generator = CommandGenerator(names, location_names, placement_location_names, room_names, object_names,
                                 object_categories_plural, object_categories_singular)
        
        self.generate_client = self.create_client(GeneratePlan, 'gpsr_planning')
        
        self.num_commands = 10
        self.predef_commands = [
            "tell me what is the heaviest dish on the desk"
        ]
        
        self.command_event = Event()
        self.finish_command = Event()
        
    def on_press(self, key):
        self.command_event.wait()
        
        if key == keyboard.Key.esc:
            self.get_logger().info('Exiting...')
            self.finish_command.set()
            return False
        elif type(key) == keyboard.KeyCode and key.char == 'a':
            self.get_logger().info(f'Command: {self.command}')
            curr_time = time.time()
            _ = self.send_command(self.command)
            elapsed_time = time.time() - curr_time
            self.get_logger().info(f'Plan generated')
            self.get_logger().info(f'Elapsed time: {elapsed_time:.2f} s\n')
            
            self.finish_command.set()
            self.finish_command.clear()
        
    def send_batch(self):
        self.get_logger().info('Press "a" to generate a command')
        self.get_logger().info('Press "esc" to exit')
        
        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()

        for i in range(self.num_commands):
            self.command = ''
            if i < len(self.predef_commands):
                self.command = self.predef_commands[i]
            else:
                self.command = self.generator.generate_command_start(cmd_category="")
                
            self.command_event.set()
            self.finish_command.wait()
            
            if not listener.running:
                break
            
            self.command_event.clear()
                    
    def send_command(self, command):
        self.generate_client.wait_for_service()
        future = self.generate_client.call_async(GeneratePlan.Request(command=command))
        rclpy.spin_until_future_complete(self, future)
        return future.result().bt_xml
    
    def read_data(self, file_path):
        with open(file_path, 'r') as file:
            data = file.read()
        return data
        
    def parse_names(self, data):
        parsed_names = re.findall(r'\|\s*([A-Za-z]+)\s*\|', data, re.DOTALL)
        parsed_names = [name.strip() for name in parsed_names]

        if parsed_names:
            return parsed_names[1:]
        else:
            warnings.warn("List of names is empty. Check content of names markdown file")
            return []


    def parse_locations(self, data):
        parsed_locations = re.findall(r'\|\s*([0-9]+)\s*\|\s*([A-Za-z,\s, \(,\)]+)\|', data, re.DOTALL)
        parsed_locations = [b for (a, b) in parsed_locations]
        parsed_locations = [location.strip() for location in parsed_locations]

        parsed_placement_locations = [location for location in parsed_locations if location.endswith('(p)')]
        parsed_locations = [location.replace('(p)', '') for location in parsed_locations]
        parsed_placement_locations = [location.replace('(p)', '') for location in parsed_placement_locations]
        parsed_placement_locations = [location.strip() for location in parsed_placement_locations]
        parsed_locations = [location.strip() for location in parsed_locations]

        if parsed_locations:
            return parsed_locations, parsed_placement_locations
        else:
            warnings.warn("List of locations is empty. Check content of location markdown file")
            return []


    def parse_rooms(self, data):
        parsed_rooms = re.findall(r'\|\s*(\w+ \w*)\s*\|', data, re.DOTALL)
        parsed_rooms = [rooms.strip() for rooms in parsed_rooms]

        if parsed_rooms:
            return parsed_rooms[1:]
        else:
            warnings.warn("List of rooms is empty. Check content of room markdown file")
            return []


    def parse_objects(self, data):
        parsed_objects = re.findall(r'\|\s*(\w+)\s*\|', data, re.DOTALL)
        parsed_objects = [objects for objects in parsed_objects if objects != 'Objectname']
        parsed_objects = [objects.replace("_", " ") for objects in parsed_objects]
        parsed_objects = [objects.strip() for objects in parsed_objects]

        parsed_categories = re.findall(r'# Class \s*([\w,\s, \(,\)]+)\s*', data, re.DOTALL)
        parsed_categories = [category.strip() for category in parsed_categories]
        parsed_categories = [category.replace('(', '').replace(')', '').split() for category in parsed_categories]
        parsed_categories_plural = [category[0] for category in parsed_categories]
        parsed_categories_plural = [category.replace("_", " ") for category in parsed_categories_plural]
        parsed_categories_singular = [category[1] for category in parsed_categories]
        parsed_categories_singular = [category.replace("_", " ") for category in parsed_categories_singular]

        if parsed_objects or parsed_categories:
            return parsed_objects, parsed_categories_plural, parsed_categories_singular
        else:
            warnings.warn("List of objects or object categories is empty. Check content of object markdown file")
            return []
        
def main(args=None):
    rclpy.init(args=args)
    
    gpsr_client = GPSRGenerator()
    gpsr_client.send_batch()
    
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()