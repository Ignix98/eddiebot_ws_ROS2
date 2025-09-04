import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_eddiebot = get_package_share_directory('eddiebot_gazebo')

    # Añadir URDF y Meshes al GZ_SIM_RESOURCE_PATH
    #gazebo_models_path, _ = os.path.split(pkg_eddiebot)
    #os.environ["GZ_SIM_RESOURCE_PATH"] = os.environ.get("GZ_SIM_RESOURCE_PATH", "") + os.pathsep + gazebo_models_path

    # Argumentos
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='eddiebot.urdf',
        description='URDF model file name'
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='world_default.sdf',
        description='World file name'
    )

    # Paths
    urdf_file_path = PathJoinSubstitution([
        pkg_eddiebot, 'urdf', LaunchConfiguration('model')
    ])

    rviz_config_path = os.path.join(pkg_eddiebot, 'rviz', 'rviz.rviz')

    # Lanzar mundo
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_eddiebot, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
            'world': LaunchConfiguration('world')
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro', ' ', urdf_file_path]),
                value_type=str
            ),
            'use_sim_time': True
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Spawn robot en Gazebo
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "my_robot",
            "-topic", "robot_description",
            "-x", "0", "-y", "0", "-z", "0.3", "-Y", "0.0"
        ],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    # Puente GZ ↔ ROS
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V"
        ],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}]
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro', ' ', urdf_file_path]),
                value_type=str
            ),
            'use_sim_time': True
        }],
        output='screen'
    )


    # Spawner para joint_state_broadcaster (necesario para TF dinámicos)
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    

    # Lanzamiento
    ld = LaunchDescription()
    ld.add_action(rviz_arg)
    ld.add_action(model_arg)
    ld.add_action(world_arg)
    ld.add_action(world_launch)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_urdf_node)
    ld.add_action(gz_bridge_node)
    ld.add_action(rviz_node)
    ld.add_action(controller_manager_node)
    ld.add_action(joint_state_broadcaster_spawner)

    return ld

