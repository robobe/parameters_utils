
#!/usr/bin/env python3

from rclpy.node import Node
from std_srvs.srv import Trigger
import yaml
import os

TOPIC = "dump"

class ParameterManagerEx():
    def __init__(self, node: Node, root=""):
        self.node = node
        self._data = None
        self.root = root
        self.location = self.node.get_parameter("param_yaml_full_path").value
        self._load_parameters(root)
        
        self._init_service()


    def _open_parameters(self, root):
        if not os.path.exists(self.location):
            self.node.get_logger().error(f"Parameter file not found: {self.location}")
            return
        
        with open(self.location, "r") as file:
            data = yaml.safe_load(file)

        if root:
            self.node.get_logger().error(f"root name ::: {root}")
            if root in data:
                data = data[root]
            else:
                self.node.get_logger().warning(f"root {root} node found, parameters not update from this file" )
                data = None
            return data
            
    def _load_parameters(self, root):
        data = self._open_parameters(root)
        if not data:
            return
        
        self.append_yaml_to_node(data)


    def _init_service(self):
        self.dump_srv = self.node.create_service(Trigger, 
                                self.node.get_name() + "/" + TOPIC, 
                                self.save_tracking_parameters)
        
    def _create_node_handler(self, key, value):
        self.node.get_logger().error(f"--- {key} -- {value}")
        if self.node.has_parameter(key):
            #TODO: check if there beater way
            self.node.get_parameter(key)._value = value
        else:
            self.node.declare_parameter(key, value)
        
    def traverse_yaml(self, data, parent_key="", handler=None):
        if isinstance(data, dict):
            for key, value in data.items():
                new_key = f"{parent_key}.{key}" if parent_key else key
                self.traverse_yaml(value, new_key, handler)
        elif isinstance(data, list):
            for index, item in enumerate(data):
                new_key = f"{parent_key}"
                self.traverse_yaml(item, new_key, handler)
        else:
            # self.node[parent_key] = data
            handler(parent_key, data)

    def append_yaml_to_node(self, data):
        self.traverse_yaml(data, handler=self._create_node_handler)

    def save_tracking_parameters(self, request: Trigger.Request, response: Trigger.Response):
        #TODO: how to lock between process in python 

        with open(self.location, "r") as file:
            data = yaml.safe_load(file)

        node_part = None
        if self.root:
            if self.root in data:
                node_part = data[self.root]

        if not node_part:
            response.success = False
            response.message = f"root: {self.root} not found for saving"
            return response
        
        self.get_tracking_parameters(node_part)

        # TODO: add exception to logger
        with open(self.location, "w") as file:
            yaml.dump(data, file, default_flow_style=False, sort_keys=False, indent=4)
        response.success = True
        response.message = f"save parameters to {self.location} "
        return response
    
    def get_tracking_parameters(self, data):
        # holed all keys from yaml
        keys = []
        def handler(key, value):
            keys.append(key)

        self.traverse_yaml(data, handler=handler)

        # hold all param name to update
        for key in keys:
            items = key.split(".")
            key_to_update = items[-1]
            key_path = items[:-1]
            source = data
            for item in key_path:
                source = source[item]
            
            try:
                source[key_to_update] = self.node.get_parameter(key).value
            except Exception as err:
                self.node.get_logger().error(f"Failed to update yaml key: {key_to_update} with param name; {key}")


        
