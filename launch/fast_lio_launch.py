import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the HOME environment variable for save_directory
    home_dir = os.environ.get('HOME', '')

    # Declare launch arguments for flexibility
    return LaunchDescription([
        DeclareLaunchArgument('save_directory', 
                              default_value=os.path.join(home_dir, 'ros2_ws/src/SC-PGO/PCD/'),
                              description='Directory for saving data'),

        DeclareLaunchArgument('scan_line', default_value='64', description='Number of scan lines'),
        DeclareLaunchArgument('mapping_skip_frame', default_value='1', description='Mapping skip frame'),
        DeclareLaunchArgument('minimum_range', default_value='0.5', description='Minimum range for valid points'),
        DeclareLaunchArgument('mapping_line_resolution', default_value='0.4', description='Mapping line resolution'),
        DeclareLaunchArgument('mapping_plane_resolution', default_value='0.8', description='Mapping plane resolution'),
        DeclareLaunchArgument('keyframe_meter_gap', default_value='0.2', description='Keyframe gap in meters'),
        DeclareLaunchArgument('sc_dist_thres', default_value='1.0', description='Scan context distance threshold'),
        DeclareLaunchArgument('sc_max_radius', default_value='80.0', description='Scan context max radius'),

        # Create the alaserPGO node with remappings and parameters
        Node(
            package='aloam_velodyne',
            executable='alaserPGO',
            name='alaserPGO',
            output='screen',
            parameters=[{
                'scan_line': LaunchConfiguration('scan_line'),
                'mapping_skip_frame': LaunchConfiguration('mapping_skip_frame'),
                'minimum_range': LaunchConfiguration('minimum_range'),
                'mapping_line_resolution': LaunchConfiguration('mapping_line_resolution'),
                'mapping_plane_resolution': LaunchConfiguration('mapping_plane_resolution'),
                'keyframe_meter_gap': LaunchConfiguration('keyframe_meter_gap'),
                'sc_dist_thres': LaunchConfiguration('sc_dist_thres'),
                'sc_max_radius': LaunchConfiguration('sc_max_radius'),
                'save_directory': LaunchConfiguration('save_directory')
            }],
            remappings=[
                ('/velodyne_cloud_registered_local', '/cloud_registered_local'),
                ('/aft_mapped_to_init', '/aft_mapped_to_init')
            ]
        ),

        # Optionally, if you want to use RViz for visualization in ROS2
        Node(
            package='rviz2',  # Correct ROS2 RViz package
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=[
                '-d', os.path.join(
                    os.path.dirname(os.path.realpath(__file__)), 'rviz_cfg', 'aloam_velodyne.rviz'
                )
            ]
        ),
    ])
