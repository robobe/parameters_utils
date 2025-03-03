import pytest
import yaml
from rclpy.node import Node
from rclpy.parameter import Parameter
import rclpy

from parameters_manager_ex import ParameterManagerEx

YAML_DATA = """
    preset: "low"
    width: 640
    height: 480
    high:
        fps: 5
        bitrate: 200
        iframe_interval: 5
        vbv: 200
    low:
        fps: 5
        bitrate: 200
        iframe_interval: 5
        vbv: 200
        """

@pytest.fixture
def ros_node():
    """Fixture to initialize and shutdown a ROS 2 node for testing."""
    rclpy.init()
    node = Node('test_node')
    yield node
    node.destroy_node()
    rclpy.shutdown()

@pytest.fixture
def temp_yaml_file():
    """Creates a temporary YAML file for testing."""
    data = yaml.safe_load(YAML_DATA)
    return data

# def test_parse(temp_yaml_file):
#     data = {}
#     ex = ParameterManagerEx(data)
#     ex.traverse_yaml(temp_yaml_file)


#     print("111111111111111111111111111111111")
#     print(ex.node)
#     success = True
#     if "preset" not in ex.node:
#         success &= False

#     if "high.fps" not in ex.node:
#         success &= False

#     assert success

# def test_parse_2_param(temp_yaml_file, ros_node: Node):
#     ex = ParameterManagerEx(ros_node)
#     ex.append_yaml_to_node(temp_yaml_file)


#     print(ex.node)
#     success = True
#     if not ros_node.has_parameter("preset"):
#         success &= False

#     if not ros_node.has_parameter("low.fps"):
#         success &= False

#     assert success

def test_update(temp_yaml_file, ros_node: Node):
    """
    update node parameter
    change param value
    get yaml back
    """
    ex = ParameterManagerEx(ros_node)
    ex.append_yaml_to_node(temp_yaml_file)

    ros_node.declare_parameter("param_not_to_dump", "failed if dump")
    ros_node.get_parameter("preset")._value = "high"
    ros_node.get_parameter("high.fps")._value = 20
    ex.get_and_update_tracking_parameters(temp_yaml_file)

    success = True
    if temp_yaml_file["preset"] != "high":
        success &= False

    if temp_yaml_file["high"]["fps"] != 20:
        success &= False
    with open("/tmp/output.yaml", "w") as file:
        yaml.dump(temp_yaml_file, file, default_flow_style=False, sort_keys=False, indent=4)
    assert success
# colcon test --packages-select parameters_manager_ex --pytest-args -k test_update --event-handlers console_direct+
