from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

launch_args = []

def launch_setup(context):
    pass

def generate_launch_description():
    ld = LaunchDescription(launch_args)
    opfunc = OpaqueFunction(function = launch_setup)
    ld.add_action(opfunc)

    other_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur5e_moveit'),
                'launch/test/test.trajectory_subscriber.launch.py')
        )
    )

    ld.add_action(other_launch_file)

    return ld