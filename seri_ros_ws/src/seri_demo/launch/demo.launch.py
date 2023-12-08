# pull in some Python launch modules.
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    OpaqueFunction,
)

def launch_setup(context, *args, **kwargs):
    '''
    Returns a list of nodes to launch.

    Args:
        context (): The context object for the launch.

    Returns:
        List[Node]: A list of nodes to launch.
    '''

    # initialize arguments passed to the launch file
    
    
    demo_action = Node(
        package="seri_demo",
        executable="demo_action.py",
    )

    # all the nodes to launch
    nodes_to_start = [demo_action]

    return nodes_to_start


def generate_launch_description():
    '''
    Function to generate a LaunchDescription object.

    Returns:
        LaunchDescription: A LaunchDescription object.
    '''

    launch_description = LaunchDescription([OpaqueFunction(function=launch_setup)])

    return launch_description