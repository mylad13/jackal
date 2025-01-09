from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')

    # Declare namespace argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    filepath_config_twist_mux = PathJoinSubstitution(
        [FindPackageShare('jackal_control'), 'config', 'twist_mux.yaml']
    )

    filepath_config_interactive_markers = PathJoinSubstitution(
        [FindPackageShare('jackal_control'), 'config', 'teleop_interactive_markers.yaml']
    )

    node_interactive_marker_twist_server = Node(
        package='interactive_marker_twist_server',
        executable='marker_server',
        name='twist_server_node',
        namespace=namespace,
        remappings={('cmd_vel', 'twist_marker_server/cmd_vel')},
        parameters=[filepath_config_interactive_markers],
        output='screen',
    )

    node_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        namespace=namespace,
        output='screen',
        remappings={('/cmd_vel_out', '/jackal_velocity_controller/cmd_vel_unstamped')},
        parameters=[filepath_config_twist_mux]
    )

    ld = LaunchDescription()
    ld.add_action(namespace_arg)
    ld.add_action(node_interactive_marker_twist_server)
    ld.add_action(node_twist_mux)
    return ld
