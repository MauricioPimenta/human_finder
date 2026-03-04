from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
			default_value='false',
			description='Whether to use synchronous SLAM'
		),

		DeclareLaunchArgument(
			'spawn_human',
			default_value='true',
			description='Spawn randomized human model in Gazebo'
		),

		DeclareLaunchArgument(
			'launch_human_detector',
			default_value='false',
			description='Launch human_detector package'
		),

		DeclareLaunchArgument(
			'human_min_x',
			default_value='-20.0',
			description='Minimum x for random human pose'
		),

		DeclareLaunchArgument(
			'human_max_x',
			default_value='20.0',
			description='Maximum x for random human pose'
		),

		DeclareLaunchArgument(
			'human_min_y',
			default_value='-20.0',
			description='Minimum y for random human pose'
		),

		DeclareLaunchArgument(
			'human_max_y',
			default_value='20.0',
			description='Maximum y for random human pose'
		),

		#
		# Launch the Clearpath simulation environment
		#
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				os.path.join(
					get_package_share_directory('human_finder'),
					'launch',
					'simulation.launch.py'
				)
			),
			launch_arguments={
				'rviz': 'false',
				'world': 'office',
				'setup_path': clearpath_setup_path,
				'use_sim_time': use_sim_time,
				'auto_start': 'true'
			}.items()
		),

		#
		# Spawn randomized human model in Gazebo
		#
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				os.path.join(
					get_package_share_directory('human_finder'),
					'launch',
					'spawn_human_gz.launch.py'
				)
			),
			launch_arguments={
				'spawn_human': LaunchConfiguration('spawn_human'),
				'world': 'office',
				'human_name': 'male_visitor_on_phone',
				'human_min_x': LaunchConfiguration('human_min_x'),
				'human_max_x': LaunchConfiguration('human_max_x'),
				'human_min_y': LaunchConfiguration('human_min_y'),
				'human_max_y': LaunchConfiguration('human_max_y'),
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
				'namespace': namespace,
    			'config': os.path.join(
					get_package_share_directory('human_finder'),
					'config',
					'nav2_exploration.rviz'
				)
			}.items()
		),

		#
		# Launch SLAM
		#
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				os.path.join(
					get_package_share_directory('human_finder'),
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
					get_package_share_directory('human_finder'),
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
		# Publish static TF alias for the camera frame expected by human_detector
		#
		Node(
			package='tf2_ros',
			executable='static_transform_publisher',
			name='camera_optical_frame_alias',
			namespace=namespace,
			arguments=[
				'0', '0', '0',
				'0', '0', '0',
				[namespace, '/robot/base_link/camera_0'],
				'camera_0_color_optical_frame',
			],
			remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
			output='screen',
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
			condition=IfCondition(LaunchConfiguration('launch_human_detector')),
			launch_arguments={
				'use_sim_time': use_sim_time,
				'namespace': namespace
			}.items()
		),

		#
		# Launch human_finder
		#
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				os.path.join(
					get_package_share_directory('human_finder'),
					'launch',
					'human_finder.launch.py'
				)
			),
			launch_arguments={
				'use_sim_time': use_sim_time,
				'namespace': namespace
			}.items()
		),

		
	])
