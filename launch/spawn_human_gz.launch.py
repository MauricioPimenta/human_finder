import math
import random

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument(
        'spawn_human',
        default_value='true',
        description='Spawn MaleVisitorOnPhone model in Gazebo'),
    DeclareLaunchArgument(
        'world',
        default_value='office',
        description='Gazebo world name'),
    DeclareLaunchArgument(
        'human_name',
        default_value='male_visitor_on_phone',
        description='Model name in Gazebo'),
    DeclareLaunchArgument(
        'human_z',
        default_value='0.1',
        description='Human z coordinate'),
    DeclareLaunchArgument(
        'human_min_x',
        default_value='-6.0',
        description='Minimum random x'),
    DeclareLaunchArgument(
        'human_max_x',
        default_value='6.0',
        description='Maximum random x'),
    DeclareLaunchArgument(
        'human_min_y',
        default_value='-4.0',
        description='Minimum random y'),
    DeclareLaunchArgument(
        'human_max_y',
        default_value='4.0',
        description='Maximum random y'),
]


def _spawn_human(context, *args, **kwargs):
    if LaunchConfiguration('spawn_human').perform(context).lower() != 'true':
        return []

    world = LaunchConfiguration('world').perform(context)
    name = LaunchConfiguration('human_name').perform(context)
    z = float(LaunchConfiguration('human_z').perform(context))
    min_x = float(LaunchConfiguration('human_min_x').perform(context))
    max_x = float(LaunchConfiguration('human_max_x').perform(context))
    min_y = float(LaunchConfiguration('human_min_y').perform(context))
    max_y = float(LaunchConfiguration('human_max_y').perform(context))

    x = random.uniform(min_x, max_x)
    y = random.uniform(min_y, max_y)
    yaw = random.uniform(-math.pi, math.pi)

    model_sdf = f"""
<sdf version='1.9'>
  <include>
    <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/MaleVisitorOnPhone</uri>
  </include>
</sdf>
""".strip()

    return [
        LogInfo(
            msg=(
                f"Spawning human '{name}' in world '{world}' at "
                f"x={x:.3f}, y={y:.3f}, z={z:.3f}, yaw={yaw:.3f}"
            )
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', world,
                '-string', model_sdf,
                '-name', name,
                '-x', f'{x:.3f}',
                '-y', f'{y:.3f}',
                '-z', f'{z:.3f}',
                '-Y', f'{yaw:.3f}',
                '-allow_renaming', 'true',
            ],
            output='screen',
        )
    ]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=_spawn_human))
    return ld
