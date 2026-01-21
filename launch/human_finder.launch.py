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
	use_sync_slam = LaunchConfiguration('use_sync_slam')
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
			default_value='true',
			description='Use simulation time if true'
		),

		DeclareLaunchArgument(
			'clearpath_setup_path',
			default_value=clearpath_setup_path,
			description='Path to Clearpath setup files'
		),

		DeclareLaunchArgument(
			'use_sync_slam',
			default_value='true',
			description='Whether to use synchronous SLAM'
		),

		#
		# Launch the Clearpath simulation environment
		#
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				os.path.join(
					get_package_share_directory('clearpath_gz'),
					'launch',
					'simulation.launch.py'
				)
			),
			launch_arguments={
				'rviz': 'false',
				'world': 'office',
				'setup_path': clearpath_setup_path,
				'use_sim_time': use_sim_time,
				'x': '0.0',
				'y': '0.0',
				'yaw': '0.0',
				'auto_start': 'true',
				'generate': 'true'
			}.items()
		),
  
		#
		# Launch Rviz from clearpath 
		#
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				os.path.join(
					get_package_share_directory('clearpath_viz'),
					'launch',
					'view_navigation.launch.py'
				)
			),
			launch_arguments={
				'use_sim_time': use_sim_time,
				'namespace': namespace
			}.items()
		),

		#
		# Launch SLAM
		#
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				os.path.join(
					get_package_share_directory('clearpath_nav2_demos'),
					'launch',
					'slam.launch.py'
				)
			),
			launch_arguments={
				'use_sim_time': use_sim_time,
				'setup_path': clearpath_setup_path,
				'sync': use_sync_slam

			}.items()
		),

		# Launch Nav2 from the clearpath_nav2_demos package
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				os.path.join(
					get_package_share_directory('clearpath_nav2_demos'),
					'launch',
					'nav2.launch.py'
				)
			),
			launch_arguments={
				'use_sim_time': use_sim_time,
				'setup_path': clearpath_setup_path
			}.items()
		),

		#
		# Launch Explore Lite
		#
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				os.path.join(
					get_package_share_directory('explore_lite'),
					'launch',
					'explore.launch.py'
				)
			),
			launch_arguments={
				'use_sim_time': use_sim_time,
				'namespace': namespace
			}.items()
		),

		#
		# Launch human_detector
		#
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				os.path.join(
					get_package_share_directory('human_detector'),
					'launch',
					'human_detector.launch.py'
				)
			),
			launch_arguments={
				'use_sim_time': use_sim_time,
				'namespace': namespace
			}.items()
		),

		# Node(
		# 	package='human_finder',
		# 	executable='human_finder_node',
		# 	namespace=namespace,
		# 	parameters=[{'use_sim_time': use_sim_time}]
		# ),
	])
