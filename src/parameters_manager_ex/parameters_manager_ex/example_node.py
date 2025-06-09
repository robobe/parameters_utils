#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from parameters_manager_ex import ParameterManagerEx, PARAM_PERSIST_LOCATION
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import yaml

PERSISTS_LOCATION = "persists"
PARAMS = [
            "parameter_one",
            "persist_int",
            "persist_str",
            "persist_bool",
            "low.data1",
            "low.data2"
        ]

class MyNode(Node):
    def __init__(self):
        node_name="example_node"
        super().__init__(node_name)


        path = Path(get_package_share_directory('parameters_manager_ex')) / 'config' / PERSISTS_LOCATION / 'example_node.yaml'
        self.declare_parameter(PARAM_PERSIST_LOCATION, value=path.as_posix())
        
        # Example: declare a parameter with a specific type (e.g., integer)
        self.declare_parameter("persist_int", value=0)  # type: int
        self.declare_parameter("persist_str", value="default")  # type: str
        self.declare_parameter("persist_bool", value=False)  # type: bool
        self.declare_parameter("parameter_one", value=None)  # type: any (default)
        self.declare_parameter("low.data1", value=0)  # type: float
        self.declare_parameter("low.data2", value=0)  # type: float
       # 
        # persists
        self.param_dump_manager = ParameterManagerEx(self, self.get_fully_qualified_name())

        for data in self.get_parameters(names=PARAMS):
            self.get_logger().info(f"{data.name}-->{data.value}")


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()