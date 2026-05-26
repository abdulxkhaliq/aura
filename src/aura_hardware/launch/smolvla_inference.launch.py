from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    args = [
        DeclareLaunchArgument('checkpoint',   default_value='lerobot/smolvla_base',
                              description='HF id or local path to a SmolVLA checkpoint'),
        DeclareLaunchArgument('task',         default_value='Pick the object and place it in the target location.',
                              description='Language instruction passed to the policy'),
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
        DeclareLaunchArgument('front_to',     default_value='camera1',
                              description='Which model slot the front cam fills (camera1/2/3)'),
        DeclareLaunchArgument('wrist_to',     default_value='camera3',
                              description='Which model slot the wrist cam fills (camera1/2/3)'),
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
            'fps':    30,
            'width':  640,
            'height': 480,
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
            'fps':    30,
            'width':  640,
            'height': 480,
        }],
        remappings=[('camera/image_raw', '/wrist/image_raw')],
    )

    smolvla = Node(
        package='aura_hardware',
        executable='smolvla_inference_node',
        name='smolvla_inference_node',
        output='screen',
        parameters=[{
            'checkpoint':   LaunchConfiguration('checkpoint'),
            'task':         LaunchConfiguration('task'),
            'device':       LaunchConfiguration('device'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'max_delta':    LaunchConfiguration('max_delta'),
            'front_to':     LaunchConfiguration('front_to'),
            'wrist_to':     LaunchConfiguration('wrist_to'),
        }],
    )

    return LaunchDescription(args + [servo, front_cam, wrist_cam, smolvla])
