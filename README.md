# Parameters Manager Ex

- Allow external parameter file to be load into node context
- It's expose new service that SAVE the parameters that load from the file
- The parameters that load / insert by the manager behaved like regular node parameter


## Demo
- Example node that declare 5 parameters
  - "parameter_one",
  - "persist_int",
  - "persist_str",
  - "persist_bool"
- Create persist file at location `param_yaml_full_path` with value `/tmp/persist_example_node.yaml`
- Update `persist_int` parameter
- Run dump service to save the value into persist file


**/tmp/persist_example_node.yaml:**
```yaml title="/tmp/persist_example_node.yaml"
/example_node:
    persist_int: 10
    persist_str: Hello
    persist_bool: false
```

> [!TIP]
> persist file can contain multiple nodes persistence parameters
> each set has node name as root like `/example_node`

**example_node.py:**
```python title="example_node.py"
#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from parameters_manager_ex import ParameterManagerEx, PARAM_PERSIST_LOCATION

PARAMS = [
            "parameter_one",
            "persist_int",
            "persist_str",
            "persist_bool"
        ]

class MyNode(Node):
    def __init__(self):
        node_name="example_node"
        super().__init__(node_name)

        self.declare_parameter(PARAM_PERSIST_LOCATION, value="/tmp/persist_example_node.yaml")
        
        for param in PARAMS:
            self.declare_parameter(param)

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
```

### usage
#### without node parameters file

**run example_node:**
```bash title="run example_node"
ros2 run parameters_manager_ex example_node.py
#
[INFO] [1746543774.000413528] [example_node]: parameter_one-->None
[INFO] [1746543774.000632608] [example_node]: persist_int-->10
[INFO] [1746543774.000849661] [example_node]: persist_str-->Hello
[INFO] [1746543774.001060469] [example_node]: persist_bool-->False
```

**parameter list:**
```bash title="parameter list"
ros2 param list
#
/example_node:
  param_yaml_full_path
  parameter_one
  persist_bool
  persist_int
  persist_str
  use_sim_time
```

**get parameter:**
```bash title="get parameter"
ros2 param get /example_node persist_int 
#
Integer value is: 10
```

**set parameter:**
```bash title="set parameter"
ros2 param set /example_node persist_int 20
#
Set parameter successful
```

#### Save/Dump to persist file

**call dump service:**
```bash title="call dump service"
ros2 service call /example_node/dump std_srvs/srv/Trigger "{}"
```

```bash title="check persist file"
cat /tmp/persist_example_node.yaml
#
/example_node:
    persist_int: 20
    persist_str: Hello
    persist_bool: false

```

---

## Demo: with node parameter file

**/tmp/persist_example_node.yaml:**
```yaml title="/tmp/persist_example_node.yaml"
/example_node:
    persist_int: 20
    persist_str: Hello
    persist_bool: false
```

**example_node.yaml:**
```yaml title="example_node.yaml"
example_node:
  ros__parameters:
    parameter_one: 42
    param_yaml_full_path: /tmp/persist_example_node.yaml
    persist_int: 0
    persist_str: empty
    persist_bool: true
```

**run node with param file:**
```bash title="run node with param file"
ros2 run parameters_manager_ex example_node.py --ros-args --params-file example_node.yaml
#
[INFO] [1746546430.603983310] [example_node]: parameter_one-->42
[INFO] [1746546430.604333778] [example_node]: persist_int-->20
[INFO] [1746546430.604640229] [example_node]: persist_str-->Hello
[INFO] [1746546430.604869311] [example_node]: persist_bool-->False
```

> [!TIP]
> The data from the persist file override the param file values
     