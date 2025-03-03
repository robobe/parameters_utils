
#!/usr/bin/env python3

from rclpy.node import Node
from std_srvs.srv import Trigger
import yaml
import os

TOPIC = "dump"
LOCATION = "param_yaml_full_path"

class ParameterManagerEx():
    """
    Load and Save node parameter subset
    Load from file that defined in `param_yaml_full_path`
    """
    def __init__(self, node: Node, root: str):
        """
        root: node name or any root element to search in subset parameter file
        """
        self.node = node
        self.root = root
        self.location = self.node.get_parameter(LOCATION).value
        self._load_parameters(root)
        
        self._init_service()


    def _open_parameters(self, root):
        if not os.path.exists(self.location):
            self.node.get_logger().error(f"Parameter file not found: {self.location}")
            return
        
        with open(self.location, "r") as file:
            data = yaml.safe_load(file)

        if root:
            if root in data:
                data = data[root]
            else:
                self.node.get_logger().warning(f"root {root} node found, parameters not update from this file" )
                data = None
            return data
            
    def _load_parameters(self, root: str):
        """
        update node parameters values from subset
        """
        data = self._open_parameters(root)
        if not data:
            return
        
        self.append_yaml_to_node(data)


    def _init_service(self):
        self.dump_srv = self.node.create_service(Trigger, 
                                self.node.get_name() + "/" + TOPIC, 
                                self.save_tracking_parameters)
        
    def _create_node_handler(self, key, value):
        """
        update node parameter value
        """
        if self.node.has_parameter(key):
            #TODO: check if there beater way
            self.node.get_parameter(key)._value = value
        else:
            # self.node.declare_parameter(key, value)
            self.node.get_logger().warning(f"Parameter with name: {key} not exists in node")
        
    def traverse_yaml(self, data, parent_key="", handler=None):
        """
        recursive iterate the yaml file and update parameter for each leaf
        """
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
        """
        iterate the yaml file and update node parameters
        """
        self.traverse_yaml(data, handler=self._create_node_handler)

    def save_tracking_parameters(self, request: Trigger.Request, response: Trigger.Response):
        """
        Save subset with current values back to file
        """
        #TODO: how to lock between process in python 

        # open yaml subset 
        with open(self.location, "r") as file:
            data = yaml.safe_load(file)

        node_part = None
        if self.root:
            if self.root in data:
                node_part = data[self.root]
            else:
                response.success = False
                response.message = f"root: {self.root} not found for saving"
                return response
        else:
            node_part = data
                
        self.get_and_update_tracking_parameters(node_part)

        # TODO: add exception to logger
        with open(self.location, "w") as file:
            yaml.dump(data, file, default_flow_style=False, sort_keys=False, indent=4)
        response.success = True
        response.message = f"save parameters to {self.location} "
        return response
    
    def get_and_update_tracking_parameters(self, data):
        """
        Get all params name the defined as subset params

        item:
            sub_item:
                sub_sub_item:
                    key: data

        yaml_data[item][sub_item][sub_sub_item][key] = data
        key_to_update: hole the key name
        key_path: hold all the sub items until the key not included
        """
        # holed all keys from yaml
        keys = []
        def handler(key, value):
            keys.append(key)

        self.traverse_yaml(data, handler=handler)

        # for each param name in keys update yaml leaf value
        for key in keys:
            #split parameter name by "."
            items = key.split(".")
            # get key
            key_to_update = items[-1]
            # get sub item path (the path to the key)
            key_path = items[:-1]
            source = data
            # iterate over yaml until to key to update
            # check example in method description
            for item in key_path:
                source = source[item]
            
            try:
                # update yaml data from parameter value
                source[key_to_update] = self.node.get_parameter(key).value
            except Exception as err:
                self.node.get_logger().error(f"Failed to update yaml key: {key_to_update} with param name; {key}")


        
