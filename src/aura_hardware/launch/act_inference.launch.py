from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    args = [
        DeclareLaunchArgument('checkpoint', default_value='',
                              description='Local path to an ACT pretrained_model dir (required)'),
        DeclareLaunchArgument('task',         default_value='Pick up the cube',
                              description='Ignored by ACT, retained for command compatibility'),
        DeclareLaunchArgument('device',       default_value='cuda',
                              description='Torch device (cuda or cpu)'),
        DeclareLaunchArgument('publish_rate', default_value='30.0',
                              description='Control loop rate in Hz'),
        DeclareLaunchArgument('max_delta',    default_value='0.02',
                              description='Per-joint max step in radians (safety clamp)'),
        DeclareLaunchArgument('front_device', default_value='0',
                              description='v4l2 index for the front (Lenovo FHD) camera'),
        DeclareLaunchArgument('wrist_device', default_value='6',
                              description='v4l2 index for the wrist (HD camera) camera'),
        DeclareLaunchArgument('joint_units',  default_value='auto',
                              description="'auto' | 'rad' | 'deg' — unit of /joint_states"),
    ]

    servo = Node(
        package='aura_hardware',
        executable='servo_driver',
        name='servo_driver',
        output='screen',
        parameters=[{
            'port':     '/dev/ttyACM0',
            'baudrate': 1000000,
            'torque':   True,
        }],
    )

    front_cam = Node(
        package='aura_hardware',
        executable='camera_node',
        name='front_camera',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('front_device'),
            'fps':    30, 'width': 640, 'height': 480,
        }],
        remappings=[('camera/image_raw', '/front/image_raw')],
    )

    wrist_cam = Node(
        package='aura_hardware',
        executable='camera_node',
        name='wrist_camera',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('wrist_device'),
            'fps':    30, 'width': 640, 'height': 480,
        }],
        remappings=[('camera/image_raw', '/wrist/image_raw')],
    )

    act = Node(
        package='aura_hardware',
        executable='act_inference_node',
        name='act_inference_node',
        output='screen',
        parameters=[{
            'checkpoint':   LaunchConfiguration('checkpoint'),
            'device':       LaunchConfiguration('device'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'max_delta':    LaunchConfiguration('max_delta'),
            'joint_units':  LaunchConfiguration('joint_units'),
        }],
    )

    return LaunchDescription(args + [servo, front_cam, wrist_cam, act])
