from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Configs
    config_jackal_ekf = PathJoinSubstitution(
        [FindPackageShare('jackal_control'),
         'config',
         'localization.yaml'],
    )

    config_imu_filter = PathJoinSubstitution(
        [FindPackageShare('jackal_control'),
         'config',
         'imu_filter.yaml'],
    )

    config_jackal_velocity_controller = PathJoinSubstitution(
        [FindPackageShare('jackal_control'),
        'config',
        'control.yaml'],
    )

    # Launch Arguments

    robot_description_command_arg = DeclareLaunchArgument(
        'robot_description_command',
        default_value=[
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('jackal_description'), 'urdf', 'jackal.urdf.xacro']
            )
        ]
    )

    robot_description_content = ParameterValue(
        Command(LaunchConfiguration('robot_description_command')),
        value_type=str
    )

    is_sim = LaunchConfiguration('is_sim', default=False)

    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value=is_sim)
    
    namespace = LaunchConfiguration('namespace')

    # Declare namespace argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )


    # Localization
    localization_group_action = GroupAction([
        # Extended Kalman Filter
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            namespace=namespace,
            output='screen',
            parameters=[config_jackal_ekf],
        ),

        # Madgwick Filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_node',
            namespace=namespace,
            output='screen',
            parameters=[config_imu_filter]
        )
    ])

    # Add the controller_manager node
    controller_manager_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=namespace,
            parameters=[config_jackal_velocity_controller],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            # condition=UnlessCondition(is_sim)
        )
    
    velocity_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        name='velocity_controller_spawner',
        namespace=namespace,
        output='screen',
        arguments=['jackal_velocity_controller'],
        parameters=[config_jackal_velocity_controller]
    )

    
    # Add the joint_state_broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        output='screen',
        arguments=['joint_state_broadcaster']
    )
    # # ROS2 Controllers
    # control_group_action = GroupAction([
    #     # ROS2 Control
    #     Node(
    #         package='controller_manager',
    #         executable='ros2_control_node',
    #         parameters=[{'robot_description': robot_description_content},
    #                     config_jackal_velocity_controller],
    #         output={
    #             'stdout': 'screen',
    #             'stderr': 'screen',
    #         },
    #         condition=UnlessCondition(is_sim)
    #     ),

    #     # Joint State Broadcaster
    #     Node(
    #         package='controller_manager',
    #         executable='spawner',
    #         arguments=['joint_state_broadcaster'],
    #         namespace=namespace,
    #         output='screen',
    #     ),

    #     # Velocity Controller
    #     Node(
    #         package='controller_manager',
    #         executable='spawner',
    #         arguments=['jackal_velocity_controller'],
    #         namespace=namespace,
    #         output='screen',
    #     )
    # ])

    ld = LaunchDescription()
    ld.add_action(robot_description_command_arg)
    ld.add_action(is_sim_arg)
    ld.add_action(namespace_arg)
    ld.add_action(localization_group_action)
    ld.add_action(controller_manager_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(velocity_controller_node)

    return ld
