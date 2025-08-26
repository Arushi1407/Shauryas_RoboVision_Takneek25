from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os

THIS_DIR = os.path.dirname(__file__)
PKG_DIR = os.path.abspath(os.path.join(THIS_DIR, '..'))

def generate_launch_description():
    # adjust these paths to your files
    world_path = os.path.expanduser('~/try/src/my_robot_description/worlds/my_world.world')
    xacro_path = os.path.expanduser('~/try/src/my_robot_description/urdf/myrobot.xacro')

    # Launch gazebo
    gazebo_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py', f'world:={world_path}'],
        shell=False
    )

    # Spawn entity after Gazebo starts (use a small Timer to allow spawn node to connect)
    spawn_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=['-file', xacro_path, '-entity', 'vision_bot','-x','5.0','-y','0.0','-z','0.0','-Y','3.14'],
    )

    # camera_saver node (saves frames to folder)
    camera_saver_node = Node(
        package='vision_recon',
        executable='camera_saver',
        name='camera_saver',
        parameters=[{
            'image_topic': '/camera/image_raw',   # change if your camera topic differs
            'out_dir': os.path.expanduser('~/try/recon_images')
        }]
    )

    # move_and_capture node (drives robot around origin)
    move_and_capture_node = Node(
        package='vision_recon',
        executable='move_and_capture',
        name='move_and_capture',
        parameters=[{
            'radius': 1.0,
            'n_views': 12,
            'odom_topic': '/odom',
            'cmd_vel_topic': '/cmd_vel',
            'save_topic': '/save_image',
            'settle_time': 1.0
        }]
    )

    # spawn only after Gazebo up — use TimerAction: spawn after 3s, and start nodes after 5s
    return LaunchDescription([
        gazebo_launch,
        TimerAction(period=3.0, actions=[spawn_cmd]),
        TimerAction(period=5.0, actions=[camera_saver_node, move_and_capture_node]),
    ])
