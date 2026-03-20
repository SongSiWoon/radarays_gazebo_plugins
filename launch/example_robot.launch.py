import os

import xacro

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('radarays_gazebo_plugins')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    # Build GAZEBO_PLUGIN_PATH so gzserver finds radarays and rmagine plugins
    # regardless of whether the workspace was sourced beforehand
    radarays_lib = os.path.join(get_package_prefix('radarays_gazebo_plugins'), 'lib', 'radarays_gazebo_plugins')
    rmagine_lib   = os.path.join(get_package_prefix('rmagine_gazebo_plugins'),  'lib', 'rmagine_gazebo_plugins')
    existing_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    plugin_path   = ':'.join(filter(None, [radarays_lib, rmagine_lib, existing_path]))

    set_gazebo_plugin_path = SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugin_path)

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

    # Gazebo server: load the sensor register plugin for the selected backend
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose': verbose,
            'paused': paused,
            'extra_gazebo_args': PythonExpression([
                '"-slibradarays_embree_sensors_gzregister.so"'
                ' if "', rmagine, '" == "embree"'
                ' else "-slibradarays_optix_sensors_gzregister.so"'
            ]),
        }.items(),
    )

    # Gazebo client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui),
    )

    # URDF for each backend (process xacro to expand macros and variables)
    urdf_cpu = os.path.join(pkg, 'urdf', 'robot_radar_cpu.urdf.xacro')
    urdf_gpu = os.path.join(pkg, 'urdf', 'robot_radar_gpu.urdf.xacro')

    robot_description_cpu = xacro.process_file(urdf_cpu).toxml()
    robot_description_gpu = xacro.process_file(urdf_gpu).toxml()

    robot_state_publisher_cpu = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_cpu,
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
            'robot_description': robot_description_gpu,
            'publish_frequency': 100.0,
        }],
        condition=IfCondition(
            PythonExpression(["'", rmagine, "' == 'optix'"])
        ),
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
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

    # Static TF: map -> odom -> base_footprint so RViz can resolve all frames
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    static_tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
    )

    return LaunchDescription([
        set_gazebo_plugin_path,
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
        joint_state_publisher,
        spawn_model,
        static_tf_map_odom,
        static_tf_odom_base,
    ])
