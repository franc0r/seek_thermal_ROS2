from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace="seekthermal",
            package='seekthermal_camera',
            executable='seekthermal_node',
            output='screen',
            parameters=[
                {"type": "seek"},
                {"camera_info_url": ""},
                {"camera_name": "seek"},
                {"frame_id": "seekpro_optical"},
                {"ffc_image": ""},
                {"cal_beta": 200.0},
                {"linear_k": -1.5276},
                {"linear_offset": 470.8979},
            ])
    ])
