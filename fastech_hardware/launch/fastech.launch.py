from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {
                    "robot_description": "<robot><ros2_control name='FastechSystem' type='system'><hardware><plugin>fastech_hardware/FastechSystem</plugin></hardware></ros2_control></robot>",
                    "controller_manager.update_rate": 50
                }
            ],
            output="screen"
        )
    ])
