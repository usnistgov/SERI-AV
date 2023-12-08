# pull in some Python launch modules.
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    '''
    Returns a list of nodes to launch.

    Args:
        context (): The context object for the launch.

    Returns:
        List[Node]: A list of nodes to launch.
    '''

    # initialize arguments passed to the launch file
    demo_action = LaunchConfiguration('demo_action').perform(context)
    
    param_file = os.path.join(
        get_package_share_directory('seri_demo'),
        'config',
        'town04_params.yaml'
    )

    vehicle_commander = Node(
        package="seri_demo",
        executable="vehicle_commander.py",
        name="hv",
        parameters=[param_file]
    )
    
    demo_action = Node(
        package="seri_demo",
        executable="demo_action.py",
        # parameters=[
        #     {"demo_action": demo_action}
        # ]
    )

    # all the nodes to launch
    nodes_to_start = [vehicle_commander]

    return nodes_to_start


def generate_launch_description():
    '''
    Function to generate a LaunchDescription object.

    Returns:
        LaunchDescription: A LaunchDescription object.
    '''

    demo_arg = DeclareLaunchArgument(
        "demo_action",
        default_value='')

    declared_arguments = []
    declared_arguments.append(demo_arg)

    launch_description = LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

    return launch_description