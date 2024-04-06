import os

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():

    insight_capture_cpp_lib_path = os.path.join(get_package_prefix("insight_capture_cpp_lib"), "lib", "libinsight_capture_cpp_lib.so")
    ip = LaunchConfiguration('ip', default="127.0.0.1")
    port = LaunchConfiguration('port', default="23")
    user = LaunchConfiguration('user', default="admin")
    pw = LaunchConfiguration('pw', default="")
    lib_path = LaunchConfiguration('lib_path', default=insight_capture_cpp_lib_path)

    arg_ip = DeclareLaunchArgument("ip", default_value="127.0.0.1")
    arg_port = DeclareLaunchArgument("port", default_value="23")
    arg_user = DeclareLaunchArgument("user", default_value="admin")
    arg_pw = DeclareLaunchArgument("pw", default_value="")
    arg_lib_path = DeclareLaunchArgument("lib_path", default_value=insight_capture_cpp_lib_path)

    isight_capture = Node(
        package="insight_capture",
        executable="insight_capture",
        name="insight_capture",
        output="screen",
        emulate_tty=True,
        remappings=[
            ("/camera", "/image"),
        ],
        parameters=[{
            "camera_ip": ip,
            "camera_port": port,
            "camera_user": user,
            "camera_pw": pw,
            "lib_path": lib_path,
        }]
    )

    return LaunchDescription([
        arg_ip,
        arg_port,
        arg_user,
        arg_pw,
        arg_lib_path,

        isight_capture,
    ])
