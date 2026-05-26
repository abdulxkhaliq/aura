#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import pybullet as p
import pybullet_data
from ament_index_python.packages import get_package_share_directory

class DigitalTwinNode(Node):
    def __init__(self):
        super().__init__('digital_twin_node')

        urdf_path = os.path.join(
            get_package_share_directory('aura_description'),
            'urdf', 'so_arm101.urdf',
        )

        self._client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.resetDebugVisualizerCamera(
            cameraDistance=0.6,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0.0, 0.0, 0.2],
        )

        self._robot = p.loadURDF(urdf_path, useFixedBase=True)

        self._joint_map: dict[str, int] = {}
        for i in range(p.getNumJoints(self._robot)):
            info = p.getJointInfo(self._robot, i)
            if info[2] != p.JOINT_FIXED:
                self._joint_map[info[1].decode()] = i

        self.get_logger().info(
            f'Digital twin ready — mapped joints: {list(self._joint_map.keys())}'
        )

        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

    def _js_cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            idx = self._joint_map.get(name)
            if idx is not None:
                p.resetJointState(self._robot, idx, pos)

    def destroy_node(self) -> None:
        p.disconnect(self._client)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DigitalTwinNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
