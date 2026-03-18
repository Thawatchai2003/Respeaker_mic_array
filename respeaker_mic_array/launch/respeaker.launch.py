from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='respeaker_mic_array',
            executable='audio_node',
            name='audio_node',
            output='screen'
        ),

        Node(
            package='respeaker_mic_array',
            executable='graph_node',
            name='graph_node',
            output='screen'
        ),

        Node(
            package='respeaker_mic_array',
            executable='audio_listener_node',
            name='audio_listener_node',
            output='screen',
            parameters=[{
                "sample_rate": 16000,
                "chunk_size": 1024,
                "use_channel": 0,
                "debug_log": True,

                "mode": 2,
                "auto_scan_interval_sec": 0.5,
                "auto_switch_threshold_rms": 300.0,
                "switch_margin_ratio": 1.15,
                "normalize_output": False,
            }]
        ),
    ])
