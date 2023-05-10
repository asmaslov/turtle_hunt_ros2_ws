from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        parameters=[
            {'background_r': 46},
            {'background_g': 139},
            {'background_b': 87}
        ]
    )

    world_node = Node(
        package='turtle_hunt_py_pkg',
        executable='world_node',
        parameters=[
            {'auto_generate': True},
            {'period': 3.0},
            {'visible_range': 3.0}
        ]
    )

    hunter_node = Node(
        package='turtle_hunt_py_pkg',
        executable='hunter_node',
        parameters=[
            {'patrol_speed': 2.0},
            {'linear_v_min': 1.0},
            {'linear_v_max': 5.0},
            {'linear_a_max': 3.0}
        ]
    )

    detector_node = Node(
        package='turtle_hunt_py_pkg',
        executable='detector_node',
        parameters=[
            {'error': 0.1}
        ]
    )

    operator_node = Node(
        package='turtle_hunt_py_pkg',
        executable='operator_node'
    )

    ld.add_action(turtlesim_node)
    ld.add_action(world_node)
    ld.add_action(hunter_node)
    ld.add_action(detector_node)
    ld.add_action(operator_node)

    return ld
