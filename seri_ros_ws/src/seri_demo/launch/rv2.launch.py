# pull in some Python launch modules.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    # Loading parameters from a file
    node_params = os.path.join(
        get_package_share_directory('seri_demo'),
        'config',
        'town04_params.yaml'
    )
    # vehicle1_node = Node(
    #     package="seri_demo",
    #     executable="vehicle.py",
    #     name="vehicle1_commander",
    #     parameters=[node_params],
    # )

    rv1_cmd = ExecuteProcess(
        cmd=[[
            f'ROS_DOMAIN_ID=1 ros2 run seri_demo vehicle.py --ros-args -r __node:=rv1 -p vehicle_id:="rv1" --params-file {node_params}',
        ]],
        shell=True
    )
    
    rv2_cmd = ExecuteProcess(
        cmd=[[
            f'ROS_DOMAIN_ID=2 ros2 run seri_demo vehicle.py --ros-args -r __node:=rv2 -p vehicle_id:="rv2" --params-file {node_params}',
        ]],
        shell=True
    )
    
    hv = ExecuteProcess(
        cmd=[[
            f'ROS_DOMAIN_ID=3 ros2 run seri_demo vehicle.py --ros-args -r __node:=hv -p vehicle_id:="hv" --params-file {node_params}',
        ]],
        shell=True
    )

    # ld.add_action(rv1_cmd)
    ld.add_action(rv2_cmd)
    # ld.add_action(gotogoal_v3)
    return ld
