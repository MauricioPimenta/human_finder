from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Get the clearpath setup path using the HOME environment variable
    clearpath_setup_path = os.path.join(os.environ['HOME'], 'clearpath')

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='a200_0000',
            description='Robot Namespace'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation time if true'
        ),
        
        DeclareLaunchArgument(
			'clearpath_setup_path',
			default_value=clearpath_setup_path,
			description='Path to Clearpath setup files'
		),

		# Launch the Clearpath simulation environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('clearpath_gz'),
                    'launch',
                    'simulation.launch.py'
                )
            ),
            launch_arguments={
                'rviz': 'true',
                'world': 'office',
                'setup_path': clearpath_setup_path,
                'use_sim_time': use_sim_time,
                'x': '0.0',
				'y': '0.0',
				'z': '0.0',
				'yaw': '0.0',
				'auto_start': 'true',
				'generate': 'true'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                )
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time
            }.items()
        ),

        Node(
            package='explore_lite',
            executable='explore',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='human_detector',
            executable='human_detector_node',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='human_finder',
            executable='human_finder_node',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
