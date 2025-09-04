import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Ruta del paquete
    pkg_eddiebot_gazebo = get_package_share_directory('eddiebot_gazebo')

    # Añadir ruta al GZ_SIM_RESOURCE_PATH para que Ignition encuentre los archivos tipo xacro
    os.environ["GZ_SIM_RESOURCE_PATH"] = (
        os.environ.get("GZ_SIM_RESOURCE_PATH", "") + os.pathsep + pkg_eddiebot_gazebo
    )

    # Argumentos de entrada opcionales
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='world_default.sdf',
        description='Name of the Gazebo world file to load'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='eddiebot.urdf',
        description='EddieBot URDF or XACRO file'
    )

    # Ruta al modelo URDF
    urdf_file_path = PathJoinSubstitution([
        pkg_eddiebot_gazebo, 'urdf', LaunchConfiguration('model')
    ])
    
    # Lanzar el mundo de Gazebo
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_eddiebot_gazebo, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items()
    )

    # Lanzar RViz (opcional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_eddiebot_gazebo, 'rviz', 'rviz.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': True},
        ]
    )

    # Nodo para spawnear el robot en Ignition Gazebo
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "my_robot",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.5", "-Y", "0.0"
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    )

    # Publicar robot_description a partir del xacro
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file_path]),
                value_type=str
            ),
            'use_sim_time': True
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Bridge ROS 2 <-> Gazebo (Ignition)
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
        parameters=[
            {'use_sim_time': True},
        ]
    )

    # Descripción del launch
    ld = LaunchDescription()
    ld.add_action(rviz_launch_arg)
    ld.add_action(world_arg)
    ld.add_action(model_arg)
    ld.add_action(world_launch)
    ld.add_action(rviz_node)
    ld.add_action(spawn_urdf_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gz_bridge_node)

    return ld

