#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINT_ORDER = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'gripper',
]

class TwinTeleopNode(Node):
    def __init__(self):
        super().__init__('twin_teleop_node')
        self._pub = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        self.create_subscription(JointState, '/gui_joint_states', self._js_cb, 10)
        self.get_logger().info(
            'Twin teleop ready — move sliders in joint_state_publisher_gui.\n'
            'Listening on /gui_joint_states (not /joint_states) to avoid feedback loop.\n'
            'Requires servo_driver running with torque:=true.'
        )

    def _js_cb(self, msg: JointState) -> None:
        index = {name: pos for name, pos in zip(msg.name, msg.position)}
        if not all(j in index for j in JOINT_ORDER):
            return
        out = Float64MultiArray()
        out.data = [index[j] for j in JOINT_ORDER]
        self._pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = TwinTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
