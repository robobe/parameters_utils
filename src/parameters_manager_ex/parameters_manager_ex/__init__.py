
#!/usr/bin/env python3

from rclpy.node import Node
from std_srvs.srv import Trigger
import yaml

TOPIC = "save_tracking_parameters"

class ParameterManagerEx():
    def __init__(self, node: Node):
        self.node = node

    def _init_service(self):
        self.dump_srv = self.node.create_service(Trigger, 
                                self.node.get_name() + "/" + TOPIC, 
                                self.save_tracking_parameters)
        
    def _create_node_handler(self, key, value):
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


        with open("/tmp/output.yaml", "w") as file:
            yaml.dump(temp_yaml_file, file, default_flow_style=False, sort_keys=False, indent=4)
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
            
            source[key_to_update] = self.node.get_parameter(key).value

        print("------")
        print(data)

        
