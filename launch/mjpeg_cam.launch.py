from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import ComposableNodeContainer

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Camera namespace'),
]

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    mjpeg_cam_node = Node(
        name='mjpeg_cam_node',
        package="mjpeg_cam",
        namespace=namespace,
        executable="mjpeg_cam_node",
        parameters=[
            {'width' : 3840,
             'height': 2160,
             'framerate': 120,
             'device_name' : "/dev/video0"
             }
            # 4k resolution //    3840 x 2160 | 872 kb
            # 1k resolution //    1920 x 1080 | 102 kb
        ],
        output="screen"
    )
# ros2 run rqt_image_view rqt_image_view

    mjpeg_cam_view = Node(
        name='mjpeg_cam_node',
        package="rqt_image_view",
        executable="rqt_image_view"
    )

    ld = launch.LaunchDescription(ARGUMENTS)
    ld.add_action(mjpeg_cam_node)
    ld.add_action(mjpeg_cam_view)

    return ld