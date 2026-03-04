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


VALID_SPAWN_RECTANGLES = [
    ((21.1323, -4.8143), (21.0802, -1.6274), (16.6210, 2.3306), (16.9731, -1.7538)),
    ((15.0922, 0.9340), (14.9418, 7.0970), (10.4262, 7.5483), (10.59, 0.78)),
    ((2.5077, 6.2065), (-3.3491, 6.0635), (2.8242, -1.4351), (-2.8033, -1.5725)),
]


def _rectangle_bounds(rectangle):
    xs = [point[0] for point in rectangle]
    ys = [point[1] for point in rectangle]
    return min(xs), max(xs), min(ys), max(ys)


def _random_point_in_valid_region():
    weighted_regions = []
    for rectangle in VALID_SPAWN_RECTANGLES:
        min_x, max_x, min_y, max_y = _rectangle_bounds(rectangle)
        area = (max_x - min_x) * (max_y - min_y)
        weighted_regions.append(((min_x, max_x, min_y, max_y), area))

    region, _ = random.choices(weighted_regions, weights=[region[1] for region in weighted_regions], k=1)[0]
    min_x, max_x, min_y, max_y = region
    return random.uniform(min_x, max_x), random.uniform(min_y, max_y)


def _spawn_human(context, *args, **kwargs):
    if LaunchConfiguration('spawn_human').perform(context).lower() != 'true':
        return []

    world = LaunchConfiguration('world').perform(context)
    name = LaunchConfiguration('human_name').perform(context)
    z = float(LaunchConfiguration('human_z').perform(context))
    x, y = _random_point_in_valid_region()
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
