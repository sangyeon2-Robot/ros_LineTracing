from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_linetracing',
            executable='line_trace_node',
            name='line_trace_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyACM0'},
                {'baud': 115200},
                {'auto_mode': False},
                {'cam_w': 320}, {'cam_h': 240}, {'cam_fps': 20},
                {'v_fwd': 0.20}, {'v_stop': 0.00},
                {'k_w': 2.0}, {'w_max': 2.5},
                {'roi_top_ratio': 0.66},
            ]
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            emulate_tty=True,
            remappings=[('/cmd_vel', '/cmd_vel')],
        ),
    ])
