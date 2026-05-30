from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    frame_id = LaunchConfiguration('frame_id')
    heading_offset_deg = LaunchConfiguration('heading_offset_deg')

    return LaunchDescription([
        DeclareLaunchArgument(
            'frame_id', default_value='base_link',
            description='IMU header frame_id (the GPS heading describes the body).'),
        DeclareLaunchArgument(
            'heading_offset_deg', default_value='0.0',
            description='Antenna-baseline-to-forward-axis offset in degrees (NED/clockwise).'),
        Node(
            package='gps_heading_imu',
            executable='gps_heading_imu',
            name='gps_heading_imu',
            output='screen',
            parameters=[{
                'frame_id': frame_id,
                'heading_offset_deg': heading_offset_deg,
            }],
        ),
    ])
