import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('radarays_gazebo_plugins')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    # Arguments
    rmagine_arg = DeclareLaunchArgument(
        'rmagine', default_value='embree',
        description='Ray-tracing backend: embree (CPU) or optix (GPU)')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg, 'worlds', 'avz_collada.world'),
        description='Gazebo world file path')

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch Gazebo client GUI')

    paused_arg = DeclareLaunchArgument(
        'paused', default_value='false',
        description='Start Gazebo simulation paused')

    verbose_arg = DeclareLaunchArgument(
        'verbose', default_value='true',
        description='Enable verbose Gazebo output')

    x_arg = DeclareLaunchArgument('x', default_value='0')
    y_arg = DeclareLaunchArgument('y', default_value='0')
    z_arg = DeclareLaunchArgument('z', default_value='0')
    roll_arg  = DeclareLaunchArgument('roll',  default_value='0')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0')
    yaw_arg   = DeclareLaunchArgument('yaw',   default_value='0')

    rmagine = LaunchConfiguration('rmagine')
    world   = LaunchConfiguration('world')
    gui     = LaunchConfiguration('gui')
    paused  = LaunchConfiguration('paused')
    verbose = LaunchConfiguration('verbose')
    x       = LaunchConfiguration('x')
    y       = LaunchConfiguration('y')
    z       = LaunchConfiguration('z')
    roll    = LaunchConfiguration('roll')
    pitch   = LaunchConfiguration('pitch')
    yaw     = LaunchConfiguration('yaw')

    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose': verbose,
            'paused': paused,
            'extra_gazebo_args': (
                '-s libradarays_embree_sensors_gzregister.so'
            ),
        }.items(),
    )

    # Gazebo client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui),
    )

    # URDF for CPU (Embree) backend
    urdf_cpu = os.path.join(pkg, 'urdf', 'static_radar_cpu.urdf.xacro')
    urdf_gpu = os.path.join(pkg, 'urdf', 'static_radar_gpu.urdf.xacro')

    robot_state_publisher_cpu = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(urdf_cpu).read(),
            'publish_frequency': 100.0,
        }],
        condition=UnlessCondition(
            PythonExpression(["'", rmagine, "' == 'optix'"])
        ),
    )

    robot_state_publisher_gpu = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(urdf_gpu).read(),
            'publish_frequency': 100.0,
        }],
        condition=IfCondition(
            PythonExpression(["'", rmagine, "' == 'optix'"])
        ),
    )

    # Spawn model in Gazebo
    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot_model',
        arguments=[
            '-entity', 'robot_sensor',
            '-topic', 'robot_description',
            '-x', x, '-y', y, '-z', z,
            '-R', roll, '-P', pitch, '-Y', yaw,
        ],
        output='screen',
    )

    return LaunchDescription([
        rmagine_arg,
        world_arg,
        gui_arg,
        paused_arg,
        verbose_arg,
        x_arg, y_arg, z_arg,
        roll_arg, pitch_arg, yaw_arg,
        gzserver,
        gzclient,
        robot_state_publisher_cpu,
        robot_state_publisher_gpu,
        spawn_model,
    ])
