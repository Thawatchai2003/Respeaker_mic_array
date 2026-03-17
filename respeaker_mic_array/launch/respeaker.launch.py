# from launch import LaunchDescription
# from launch_ros.actions import Node
# def generate_launch_description():

#     return LaunchDescription([
#         Node(
#             package='respeaker_mic_array',
#             executable='audio_node',
#             name='audio_node',
#             output='screen'
#         ),

#         Node(
#             package='respeaker_mic_array',
#             executable='graph_node',
#             name='graph_node',
#             output='screen'
#         ),

#         Node(
#             package='respeaker_mic_array',
#             executable='audio_listener_node',
#             name='audio_listener_node',
#             output='screen'
#         ),

#     ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch args
    mic_mode = LaunchConfiguration("mic_mode")
    enable_graph = LaunchConfiguration("enable_graph")

    declare_mic_mode = DeclareLaunchArgument(
        "mic_mode",
        default_value="mode1",   # mode1 = ReSpeaker, mode2 = auto scan
        description="Microphone mode: mode1 (ReSpeaker), mode2 (auto scan)"
    )

    declare_enable_graph = DeclareLaunchArgument(
        "enable_graph",
        default_value="true",
        description="Enable graph_node true/false"
    )

    return LaunchDescription([
        declare_mic_mode,
        declare_enable_graph,

        Node(
            package='respeaker_mic_array',
            executable='audio_node',
            name='audio_node',
            output='screen',
            parameters=[{
                "mic_mode": mic_mode,
                "preferred_name": "ReSpeaker",
                "sample_rate": 16000,
                "channels": 6,
                "chunk_size": 1024,
                "debug_log": True,
            }]
        ),

        Node(
            package='respeaker_mic_array',
            executable='graph_node',
            name='graph_node',
            output='screen',
            condition=None  # ถ้ายังไม่อยากใช้เงื่อนไข ก็ปล่อยแบบนี้ก่อน
        ),

        Node(
            package='respeaker_mic_array',
            executable='audio_listener_node',
            name='audio_listener_node',
            output='screen'
        ),
    ])