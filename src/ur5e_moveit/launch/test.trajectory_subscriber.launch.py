from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

launch_args = [
    DeclareLaunchArgument(name='tcp_ip', default_value='0.0.0.0', description='TCP endpoint IP address'),
    DeclareLaunchArgument(name='tcp_port', default_value='10000', description='TCP endpoint port'),
]

def launch_setup(context):
    tcp_ip = LaunchConfiguration('tcp_ip').perform(context) # Here you'll get the runtime config value
    tcp_port = LaunchConfiguration('tcp_port').perform(context)

    server_endpoint_node = Node(
        package = 'ros_tcp_endpoint',
        executable = 'default_server_endpoint',
        name = 'server_endpoint',
        emulate_tty=True,
        parameters =[{'ROS_IP': LaunchConfiguration('tcp_ip')}, {'ROS_TCP_PORT': LaunchConfiguration('tcp_port')}]
    )

    trajectory_subscriber_node = Node(
        package = 'ur5e_moveit_py',
        executable = 'trajectory_subscriber',
        name = 'trajectory_subscriber'
    )

    return [server_endpoint_node, trajectory_subscriber_node]

def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld