import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the current directory
    current_dir = os.path.dirname(__file__)

    # Create a LaunchDescription instance
    ld = LaunchDescription()

    # Include the other launch file in the same folder
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([current_dir, '/studierbot_with_joy2vel.launch.py'])
    )
    ld.add_action(included_launch)



    # Add the USB camera node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'mjpeg2rgb',
            'camera_frame_id': 'usb_cam',
            'io_method': 'mmap',
        }],
        remappings=[('image_raw', 'usb_cam/image_raw')],


    )
    ld.add_action(usb_cam_node)




    return ld