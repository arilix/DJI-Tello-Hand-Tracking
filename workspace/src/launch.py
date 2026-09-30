
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        # Tello driver node
        Node(
            package='tello_driver',
            executable='tello_driver',
            output='screen',
            namespace='/',
            name='tello_driver',
            parameters=[
                {'tello_ip': '192.168.10.1'},
                {'cmd_port': 8889},
                {'state_port': 8890},
                {'video_port': 11111},
                {'tf_base': 'odom'},
                {'tf_drone': 'drone'},
                {'video_enabled': True},
            ],
            respawn=True
        ),

        # Tello keyboard control node
        Node(
            package='tello_control',
            executable='tello_control',
            namespace='/',
            name='tello_control',
            output='screen',
            respawn=False
        ),

        # Hand vision / tracking node
        Node(
            package='hand_vision',
            executable='hand_control_node',
            namespace='/',
            name='hand_control_node',
            output='screen',
            respawn=False,
            parameters=[
                '/home/arilix/Documents/tello/tello_ws/src/tello-ros2/workspace/install/hand_vision/share/hand_vision/config/hand_control.yaml'
            ]
        ),
    ]

    return LaunchDescription(nodes)
